#include "secrets.h"
/*
 * 固件版本: V4.9
 * 当前接线补充（STM32 V8.10 / 2026-09-19）：ESP TX->PC11(UART4_RX)，RX->PC10(UART4_TX)。
 * 原PA9/PA10信号线撤掉；VCC/EN稳定3.3V、GND共地不变；115200/8N1不变。
 * 本次只重烧STM32，此ESP固件执行代码/版本不变；下方旧PA9/PA10条目是历史说明。
 * 更新日期: 2026-09-15
 * V4.9：删除自动按路线行驶R1/R2接口，旧/api/route返回410；路线记录/保存补传与手持转发保留，接线不变。
 * V4.8：配套STM32 V8.0；串口忙时保留最新手持帧、扩充接收缓存、减少跨域预检，路线接口返回具体反馈故障。接线不变。
 * V4.7：配套STM32 V7.8，只读路线缓存补传；完整批次校验，不新增引脚或驾驶命令。
 * V4.6: R1/R2 route upload, MCU acknowledgements and browser lease; pins unchanged.
 * V4.5：接收STM32 V6.8的S1轮速诊断，API新增wheelSpeed；不新增驾驶接口。
 *   H1/E1/G2与GPIO接线不变，TX->PA10，RX->PA9，共地，VCC/EN稳定3.3V。
 *
 * V4.4：配套STM32 V6.5，接收带CRC的E1四轮累计计数，API附带16帧短缓存。
 *   2D/3D网页本地标定并估算行驶轨迹；手持H1和ESP接线保持不变。
 *   TX->PA10，RX->PA9，GND共地，VCC/EN稳定3.3V；GPIO0/2运行时保持高。
 *
 * V4.3 变更：校验H1手持帧并转发，增加车辆状态；配套STM32 V6.0和NodeMCU手持V1.0。
 * V4.2 变更：气体G2协议、18位浓度及真实单位；区分预热/故障/报警/离线。
 *   配套STM32 V5.3，接线不变（TX->PA10，RX<-PA9，共地，稳定3.3V）。
 *   温湿度编码待厂家确认，API暂返回null；旧G帧不兼容，需两端一起更新。
 *
 * V4.0 变更:
 *   - 192.168.4.1 控制界面全部改为中文
 *   - 重新设计手机和电脑端布局、工作模式按钮与连接状态
 *   - 增加距离安全等级提示，统一显示安全、注意和报警
 * V4.1 变更:
 *   - 接收STM32 USART1发送的G气体帧
 *   - /api/telemetry增加CH4、O2、CO、H2S浓度、状态、温湿度数据
 */
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include "HandheldProtocol.h"
#include "EncoderProtocol.h"
#include "WheelSpeedProtocol.h"
#include "RouteLogProtocol.h"

namespace {
const char kApName[] = "GimbalCtrl";
const char kApPassword[] = GIMBAL_WIFI_PASSWORD;
const uint16_t kUdpPort = 4210;
const size_t kUdpPacketCapacity = 128;
const size_t kTelemetryLineCapacity = 192;
const uint32_t kTelemetryTimeoutMs = 1000;
const uint32_t kGasTimeoutMs = 3000;

struct GasReading {
  uint32_t sequence;
  uint8_t channel;
  uint32_t concentrationRaw;
  uint8_t unitCode;
  uint8_t decimals;
  uint8_t status;
  uint8_t gasType;
  bool moduleOnline;
  bool valid;
  bool received;
  uint32_t receivedAtMs;
};

struct VehicleTelemetry {
  uint32_t sequence=0, receivedAtMs=0;
  uint8_t mode=0, state=0;
  int16_t pwm[4]={0,0,0,0};
  bool received=false;
};
VehicleTelemetry vehicle;

WheelSpeedFrame wheelSpeed = {};
bool wheelSpeedReceived=false;
uint32_t wheelSpeedAt=0,wheelSpeedErrors=0;
bool parseWheelSpeedLine(const char *line) {
  WheelSpeedFrame frame;
  if(!WheelSpeedFrame_Parse(line,strlen(line),&frame)){wheelSpeedErrors++;return false;}
  if(wheelSpeedReceived){
    uint32_t ds=frame.sequence-wheelSpeed.sequence,dt=frame.milliseconds-wheelSpeed.milliseconds;
    if(dt<0x80000000U&&(!ds||ds>=0x80000000U||!dt))return false;
  }
  wheelSpeed=frame;wheelSpeedAt=millis();wheelSpeedReceived=true;return true;
}

size_t appendWheelSpeedJson(char *json,size_t used,size_t capacity,uint32_t now) {
  if(used>=capacity)return capacity;
  bool online=wheelSpeedReceived&&now-wheelSpeedAt<=1000U;
  int n=snprintf(json+used,capacity-used,
    ",\"wheelSpeed\":{\"protocol\":1,\"online\":%s,\"enabled\":%s,\"valid\":%s,\"sequence\":%lu,\"ageMs\":%lu,\"errors\":%lu,\"fault\":%u,\"faultWheels\":%u,\"limited\":%u,\"sampleMs\":%u,\"cpr\":%lu,\"signs\":[%d,%d,%d,%d],\"targetRpm10\":[%d,%d,%d,%d],\"measuredRpm10\":[%d,%d,%d,%d],\"pwm\":[%d,%d,%d,%d]}",
    online?"true":"false",wheelSpeed.enabled?"true":"false",online&&wheelSpeed.valid?"true":"false",
    (unsigned long)wheelSpeed.sequence,(unsigned long)(wheelSpeedReceived?now-wheelSpeedAt:0),
    (unsigned long)wheelSpeedErrors,wheelSpeed.fault,wheelSpeed.fault_wheels,wheelSpeed.limited,
    wheelSpeed.dt_ms,(unsigned long)wheelSpeed.cpr,
    wheelSpeed.sign_mask&1?-1:1,wheelSpeed.sign_mask&2?-1:1,wheelSpeed.sign_mask&4?-1:1,wheelSpeed.sign_mask&8?-1:1,
    wheelSpeed.target[0],wheelSpeed.target[1],wheelSpeed.target[2],wheelSpeed.target[3],
    wheelSpeed.measured[0],wheelSpeed.measured[1],wheelSpeed.measured[2],wheelSpeed.measured[3],
    wheelSpeed.pwm[0],wheelSpeed.pwm[1],wheelSpeed.pwm[2],wheelSpeed.pwm[3]);
  if(n<0||(size_t)n>=capacity-used)return capacity;
  return used+(size_t)n;
}

const uint8_t kEncoderHistory = 16;
EncoderFrame encoderHistory[kEncoderHistory] = {};
uint8_t encoderHead=0, encoderSize=0;
uint32_t encoderAt=0, encoderGeneration=0, encoderBoot=0, encoderErrors=0;
bool encoderFresh=false;

bool parseEncoderLine(const char *line) {
  EncoderFrame frame;
  if(!Encoder_Parse(line,strlen(line),&frame)){encoderErrors++;return false;}
  if(encoderSize) {
    const EncoderFrame &last=encoderHistory[(encoderHead+kEncoderHistory-1)%kEncoderHistory];
    uint32_t ds=frame.sequence-last.sequence,dt=frame.milliseconds-last.milliseconds;
    // Serial is ordered: a backward MCU uptime is a reboot, not a new trajectory.
    if(dt>=0x80000000U) {encoderSize=0;encoderHead=0;encoderGeneration++;}
    else if(!ds||ds>=0x80000000U||!dt)return false;
  }
  encoderHistory[encoderHead]=frame;
  encoderHead=(encoderHead+1)%kEncoderHistory;
  if(encoderSize<kEncoderHistory)encoderSize++;
  encoderAt=millis();encoderFresh=true;
  return true;
}

size_t appendEncoderJson(char *json,size_t used,size_t capacity,uint32_t now) {
  if(used>=capacity)return capacity;
  int n=snprintf(json+used,capacity-used,
    ",\"odometry\":{\"protocol\":1,\"online\":%s,\"session\":\"%lu-%lu-%lu\",\"ageMs\":%lu,\"errors\":%lu,\"samples\":[",
    encoderFresh?"true":"false",(unsigned long)ESP.getChipId(),
    (unsigned long)encoderBoot,(unsigned long)encoderGeneration,
    (unsigned long)(encoderSize?now-encoderAt:0),(unsigned long)encoderErrors);
  if(n<0||(size_t)n>=capacity-used)return capacity;
  used+=(size_t)n;
  for(uint8_t i=0;i<encoderSize;i++){
    const EncoderFrame &e=encoderHistory[(encoderHead+kEncoderHistory-encoderSize+i)%kEncoderHistory];
    n=snprintf(json+used,capacity-used,"%s[%lu,%lu,%lu,%lu,%lu,%lu]",i?",":"",
      (unsigned long)e.sequence,(unsigned long)e.milliseconds,
      (unsigned long)e.count[0],(unsigned long)e.count[1],
      (unsigned long)e.count[2],(unsigned long)e.count[3]);
    if(n<0||(size_t)n>=capacity-used)return capacity;
    used+=(size_t)n;
  }
  if(used+3>capacity)return capacity;
  strcpy(json+used,"]}");return used+2;
}

struct Telemetry {
  uint32_t sequence;
  int32_t rollTenths;
  int32_t pitchTenths;
  int32_t distanceTenths;
  bool distanceValid;
  bool link;
  bool received;
  uint32_t receivedAtMs;
};

WiFiUDP udp;
ESP8266WebServer server(80);
Telemetry telemetry = {0, 0, 0, -10, false, false, false, 0};
GasReading gasReadings[4] = {};
char udpPacket[kUdpPacketCapacity];
char telemetryLine[kTelemetryLineCapacity];
size_t telemetryLength = 0;
bool discardTelemetryLine = false;

const char kControlPage[] PROGMEM = R"HTML(
<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#14201a">
<title>二维云台控制台</title>
<style>
:root{color-scheme:light;--page:#edf1ee;--surface:#fff;--ink:#17201b;--muted:#657069;--line:#d5dcd7;--dark:#14201a;--green:#16845b;--green-soft:#e1f3ea;--amber:#c48713;--amber-soft:#fff1ce;--red:#c8453d;--red-soft:#fbe4e1;--blue:#2f6595}
*{box-sizing:border-box}html{background:var(--page)}body{margin:0;min-width:320px;color:var(--ink);background:var(--page);font-family:"Microsoft YaHei UI","PingFang SC",Arial,sans-serif;font-size:15px;letter-spacing:0}
.shell{width:min(700px,100%);min-height:100vh;margin:auto;background:var(--surface)}
.topbar{min-height:82px;padding:16px 20px;display:flex;align-items:center;justify-content:space-between;gap:16px;color:#f4f8f5;background:var(--dark);border-bottom:3px solid var(--green)}
.brand{display:flex;align-items:center;gap:12px;min-width:0}.mark{width:38px;height:38px;flex:0 0 38px;position:relative;border:1px solid #5c7766;border-radius:50%}.mark:before,.mark:after{content:"";position:absolute;background:#46d797}.mark:before{left:18px;top:5px;width:1px;height:26px}.mark:after{left:5px;top:18px;width:26px;height:1px}
h1,h2,p{margin:0}h1{font-size:20px;font-weight:600}.subtitle{margin-top:3px;color:#aab8ae;font-size:12px}.version{margin-left:6px;color:#62dda2}
.state{min-height:34px;padding:0 11px;display:flex;align-items:center;gap:7px;flex:0 0 auto;color:#cad4cd;border:1px solid #405248;border-radius:4px;font-size:12px}.state i{width:8px;height:8px;border-radius:50%;background:var(--amber)}.state[data-online="true"] i{background:#42d28e;box-shadow:0 0 0 4px rgba(66,210,142,.14)}
main{padding:20px}.metrics{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px}.metric{min-width:0;min-height:112px;padding:15px;background:#f9fbfa;border:1px solid var(--line);border-radius:6px}.metric-label{display:flex;align-items:center;justify-content:space-between;gap:6px;color:var(--muted);font-size:12px}.metric-value{margin-top:20px;display:flex;align-items:baseline;gap:5px}.metric-value b{min-width:0;font-family:Consolas,"Microsoft YaHei UI",monospace;font-size:29px;font-weight:600;font-variant-numeric:tabular-nums}.unit{color:var(--muted);font-size:11px}.risk{padding:2px 6px;color:#56625b;background:#e8ece9;border-radius:3px;font-size:10px}.metric.distance[data-zone="safe"]{border-color:#98d5b7}.metric.distance[data-zone="safe"] .risk{color:#12633f;background:var(--green-soft)}.metric.distance[data-zone="warning"]{border-color:#e6c36c}.metric.distance[data-zone="warning"] .risk{color:#855900;background:var(--amber-soft)}.metric.distance[data-zone="danger"]{border-color:#e5a29d}.metric.distance[data-zone="danger"] .risk{color:#932d27;background:var(--red-soft)}
.controls{margin-top:25px;padding-top:20px;border-top:1px solid var(--line)}.section-head{display:flex;align-items:flex-end;justify-content:space-between;gap:14px;margin-bottom:13px}.eyebrow{margin-bottom:3px;color:var(--green);font-size:11px}.section-head h2{font-size:18px;font-weight:600}.current-mode{color:var(--muted);font-size:12px}
.mode-grid,.utility-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}.utility-grid{margin-top:10px}button{width:100%;min-width:0;min-height:68px;padding:10px 12px;cursor:pointer;color:var(--ink);background:#f8faf8;border:1px solid var(--line);border-radius:6px;font:inherit;letter-spacing:0;text-align:left}button strong,button small{display:block}button strong{font-size:15px}button small{margin-top:4px;color:var(--muted);font-size:11px;font-weight:400}button:active{transform:translateY(1px)}button.is-active{color:#fff;background:var(--green);border-color:var(--green)}button.is-active small{color:#dff7eb}.handheld{border-color:#9eb2a5}.stop{color:#9a302a;background:#fff8f7;border-color:#dfa29d}.stop small{color:#a85a54}
.notice{min-height:42px;margin-top:16px;padding:10px 12px;display:flex;align-items:center;color:#4e5d54;background:#f1f5f2;border-left:3px solid var(--green);font-size:12px}.notice.error{border-left-color:var(--red);color:#91332d;background:#fff5f4}.gas-panel{margin-top:18px;padding-top:18px;border-top:1px solid var(--line)}.gas-head{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:10px}.gas-head h2{font-size:16px;font-weight:600}.gas-head span{color:var(--muted);font-size:11px}.gas-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.gas-item{min-width:0;padding:10px 12px;background:#f8faf8;border:1px solid var(--line);border-radius:5px}.gas-item[data-valid="true"]{border-color:#98d5b7}.gas-name{display:flex;align-items:center;justify-content:space-between;color:var(--muted);font-size:11px}.gas-value{margin-top:7px;font-family:Consolas,"Microsoft YaHei UI",monospace;font-size:20px;font-weight:600}.gas-meta{margin-top:4px;color:var(--muted);font-size:10px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
footer{min-height:48px;padding:0 20px;display:flex;align-items:center;justify-content:space-between;gap:12px;color:#728078;background:#f5f7f5;border-top:1px solid var(--line);font-size:10px}
@media(max-width:500px){.topbar{padding:14px}.state{width:30px;min-height:30px;padding:0;justify-content:center}.state span{position:absolute;width:1px;height:1px;overflow:hidden;clip:rect(0 0 0 0)}main{padding:14px}.metrics{grid-template-columns:repeat(2,minmax(0,1fr))}.metric.distance{grid-column:1/-1;min-height:96px}.metric.distance .metric-value{margin-top:12px}.mode-grid,.utility-grid{gap:8px}button{min-height:64px;padding:9px}.section-head{align-items:flex-start;flex-direction:column;gap:5px}footer{padding:10px 14px;align-items:flex-start;flex-direction:column;gap:4px}}
.gas-item[data-level="normal"]{border-color:#29865f;background:rgba(41,134,95,.08)}
.gas-item[data-level="warning"],.gas-item[data-level="warmup"]{border-color:#d29625;background:rgba(210,150,37,.10)}
.gas-item[data-level="alarm"],.gas-item[data-level="fault"]{border-color:#df5148;background:rgba(223,81,72,.10)}
.gas-item[data-level="offline"]{opacity:.75}
.gas-item strong,.gas-item small{display:block}
.gas-item small{margin-top:4px;white-space:normal;overflow-wrap:anywhere}
</style>
</head>
<body>
<div class="shell">
<header class="topbar">
  <div class="brand"><span class="mark" aria-hidden="true"></span><div><h1>嵌入电团队</h1><p class="subtitle">二维云台控制台 <span class="version">V4.9</span></p></div></div>
  <div id="state" class="state" data-online="false"><i aria-hidden="true"></i><span>等待基座数据</span></div>
</header>
<main>
  <section class="metrics" aria-label="云台实时数据">
    <article class="metric"><div class="metric-label"><span>水平角</span></div><div class="metric-value"><b id="roll">--</b><span class="unit">度</span></div></article>
    <article class="metric"><div class="metric-label"><span>俯仰角</span></div><div class="metric-value"><b id="pitch">--</b><span class="unit">度</span></div></article>
    <article id="distanceCard" class="metric distance" data-zone="waiting"><div class="metric-label"><span>障碍距离</span><span id="distanceState" class="risk">等待</span></div><div class="metric-value"><b id="distance">--</b><span class="unit">厘米</span></div></article>
  </section>
  <section class="gas-panel" aria-label="车辆控制状态"><div class="gas-head"><h2>麦克纳姆底盘</h2><span>手持按键切换 · 长按软件停车</span></div><div id="vehicleStatus" class="notice">等待主控状态</div><p id="wheelOutput" class="gas-meta">1左前 / 2右前 / 3左后 / 4右后 · 数值是PWM，不是车速</p></section>
  <section class="gas-panel" aria-label="气体检测数据"><div class="gas-head"><h2>气体环境</h2><span>4MZ-HH4 · Modbus-RTU</span></div><div id="gasGrid" class="gas-grid"><div class="gas-item"><div class="gas-name"><span>CH4</span><span>等待</span></div><div class="gas-value">--</div><div class="gas-meta">等待模块数据</div></div></div></section>
  <section class="controls" aria-labelledby="modeTitle">
    <div class="section-head"><div><p class="eyebrow">扫描控制</p><h2 id="modeTitle">选择工作模式</h2></div><span id="modeText" class="current-mode">当前：体感控制</span></div>
    <div class="mode-grid">
      <button data-mode="horizontal" onclick="sendMode(this,'horizontal')"><strong>左右扫描</strong><small>水平方向往复检测</small></button>
      <button data-mode="vertical" onclick="sendMode(this,'vertical')"><strong>上下扫描</strong><small>垂直方向往复检测</small></button>
      <button data-mode="all" onclick="sendMode(this,'all')"><strong>全方位扫描</strong><small>分层覆盖检测区域</small></button>
      <button class="handheld is-active" data-mode="handheld" onclick="sendMode(this,'handheld')"><strong>体感控制</strong><small>跟随手持端姿态</small></button>
    </div>
    <div class="utility-grid"><button class="stop" data-mode="stop" onclick="sendMode(this,'stop')"><strong>停车并停止扫描</strong><small>软件停车、云台回中；硬件急停请断电</small></button></div>
  </section>
  <div id="notice" class="notice">控制台已就绪</div>
</main>
<footer><span>热点：GimbalCtrl</span><span>地址：192.168.4.1</span></footer>
</div>
<script>
var modeNames={horizontal:'左右扫描',vertical:'上下扫描',all:'全方位扫描',stop:'已停车并回中',handheld:'体感控制'};
function setActive(button,mode){document.querySelectorAll('[data-mode]').forEach(function(item){item.classList.remove('is-active')});if(mode!=='stop')button.classList.add('is-active');document.getElementById('modeText').textContent='当前：'+modeNames[mode]}
async function sendMode(button,mode){var notice=document.getElementById('notice');notice.className='notice';notice.textContent='正在发送指令...';try{var response=await fetch('/api/scan?mode='+mode);var data=await response.json();if(!response.ok)throw new Error(data.message||'指令发送失败');setActive(button,mode);notice.textContent=data.message||'指令已发送'}catch(error){notice.className='notice error';notice.textContent=error.message||'指令发送失败'}}
function updateVehicle(v) {
  const label=document.getElementById('vehicleStatus'), wheels=document.getElementById('wheelOutput');
  const states=['未解锁：等待硬件确认','小车已停用','请放平手持端，等待确认','已就绪，倾斜可行驶','正在输出驱动指令','手持失联，已撤销驱动','保护停车：排查后切换模式重试'];
  const modes=['云台体感','左右扫描','上下扫描','全方位扫描','已停止','小车体感'];
  if(!v||v.online!==true){label.textContent='车辆状态离线';label.className='notice error';wheels.textContent='PWM：--';return;}
  label.textContent=(modes[v.mode]||'未知模式')+' · '+(states[v.state]||'未知状态');
  label.className='notice'+(v.state>=5?' error':'');
  wheels.textContent='PWM '+v.pwm.map((p,i)=>(i+1)+'号 '+(p/10).toFixed(0)+'%').join(' / ')+'（非实测车速）';
  document.getElementById('modeText').textContent='当前：'+(modes[v.mode]||'未知');
  const keys=['handheld','horizontal','vertical','all','stop'];
  document.querySelectorAll('[data-mode]').forEach(el=>el.classList.toggle('is-active',el.dataset.mode===keys[v.mode]));
}
function updateDistance(value){var card=document.getElementById('distanceCard');var label=document.getElementById('distanceState');if(value==null){card.dataset.zone='waiting';label.textContent='等待';document.getElementById('distance').textContent='--';return}document.getElementById('distance').textContent=value;if(value<20){card.dataset.zone='danger';label.textContent='报警'}else if(value<40){card.dataset.zone='warning';label.textContent='注意'}else{card.dataset.zone='safe';label.textContent='安全'}}
function updateGas(gases) {
  const grid = document.getElementById('gasGrid');
  if (!grid) return;
  const rows = Array.isArray(gases) && gases.length ? gases :
    [1,2,3,4].map(channel => ({channel, name:'待识别气体'}));
  grid.replaceChildren();
  rows.forEach(g => {
    const online = g.online === true;
    const schemaOk = typeof g.unit === 'string' && typeof g.statusText === 'string';
    const valid = online && schemaOk && g.valid === true &&
      typeof g.concentration === 'number' && Number.isFinite(g.concentration);
    const allowed = ['normal','warning','alarm','fault','warmup','offline'];
    let level = online && schemaOk && allowed.includes(g.severity) ? g.severity : 'offline';
    if (online && (!schemaOk || (level === 'normal' && !valid))) level = 'fault';
    const card = document.createElement('div');
    card.className = 'gas-item';
    card.dataset.level = level;
    const value = document.createElement('strong');
    const decimals = Number.isInteger(g.decimals) && g.decimals >= 0 && g.decimals <= 3 ? g.decimals : 0;
    value.textContent = (g.name || '待识别气体') + ': ' +
      (valid ? g.concentration.toFixed(decimals) + ' ' + g.unit : '--');
    const detail = document.createElement('small');
    detail.textContent = (online ? schemaOk ? g.statusText : '需更新基座ESP' : '未连接 / 无新数据') +
      ' · 通道 ' + (g.channel || '?');
    card.append(value, detail);
    grid.append(card);
  });
}
let updating = false;
async function update() {
  if (updating) return;
  updating = true;
  const controller = new AbortController(), timeout = setTimeout(() => controller.abort(), 1500);
  const state = document.getElementById('state');
  try {
    const response = await fetch('/api/telemetry?t='+Date.now(), {cache:'no-store',signal:controller.signal});
    if (!response.ok) throw new Error('HTTP');
    const data = await response.json();
    document.getElementById('roll').textContent = data.online ? data.roll : '--';
    document.getElementById('pitch').textContent = data.online ? data.pitch : '--';
    updateDistance(data.online ? data.distance : null);
    updateGas(data.gases);
    updateVehicle(data.vehicle);
    state.dataset.online = String(data.online === true);
    state.querySelector('span').textContent = data.online ? '主控在线' : '等待主控';
  } catch (error) {
    state.dataset.online = 'false';
    state.querySelector('span').textContent = '基座通信异常';
    document.getElementById('roll').textContent = '--';
    document.getElementById('pitch').textContent = '--';
    updateDistance(null);
    updateGas([]);
    updateVehicle(null);
  } finally { clearTimeout(timeout); updating = false; }
}
setInterval(update,500);update();
</script>
</body>
</html>
)HTML";

void addCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Private-Network", "true");
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
}

#include "RouteLogBridge.h"
// Explicitly reject cached old pages; this endpoint cannot enqueue motor commands.
void sendRouteRemoved(){
  addCorsHeaders();
  server.send(410,"application/json; charset=utf-8",
    "{\"error\":\"route_drive_removed\",\"message\":\"自动按路线行驶已删除，保存和打开路线仍可使用\"}");
}

void formatTenths(char *out, size_t cap, int32_t value) {
  long whole = value / 10;
  long fraction = abs(value % 10);
  if (value < 0 && whole == 0) snprintf(out, cap, "-0.%ld", fraction);
  else snprintf(out, cap, "%ld.%ld", whole, fraction);
}

const char *gasUnit(uint8_t code) {
  switch (code) {
    case 0: return "ppm";
    case 2: return "%LEL";
    case 4: return "%VOL";
    case 6: return "mg/m3";
    case 8: return "ppb";
    default: return "";
  }
}

const char *gasName(uint8_t type) {
  switch (type) {
    case 11: return "CH4";
    case 67: return "O2";
    case 5: return "CO";
    case 52: return "H2S";
    case 45: return "EX";
    default: return "未知气体";
  }
}

bool gasStatusUsable(uint8_t status) {
  return status == 1 || status == 4 || status == 5 || status == 6 ||
         status == 11 || status == 12;
}

const char *gasStatusText(uint8_t status) {
  switch (status) {
    case 0: return "预热中";
    case 1: return "正常";
    case 2: return "数据错误";
    case 3: return "传感器故障";
    case 4: return "预警";
    case 5: return "低限报警";
    case 6: return "高限报警";
    case 7: return "访问故障";
    case 8: return "超量程";
    case 9: return "需要校准";
    case 10: return "检测超时";
    case 11: return "短时暴露报警";
    case 12: return "时间加权报警";
    case 15: return "通信故障";
    default: return "未知/保留状态";
  }
}

const char *gasSeverity(uint8_t status, bool valid) {
  if (status == 0) return "warmup";
  if (status == 4) return "warning";
  if (status == 5 || status == 6 || status == 8 || status == 11 || status == 12) return "alarm";
  if (status == 1 && valid) return "normal";
  return "fault";
}

void formatScaled(char *out, size_t cap, uint32_t raw, uint8_t decimals) {
  uint32_t divisor = 1;
  for (uint8_t i = 0; i < decimals; ++i) divisor *= 10;
  if (!decimals) snprintf(out, cap, "%lu", (unsigned long)raw);
  else snprintf(out, cap, "%lu.%0*lu", (unsigned long)(raw / divisor),
                (int)decimals, (unsigned long)(raw % divisor));
}

bool parseGasLine(const char *line) {
  // G2 has explicit module-online/usable fields. Never reinterpret old G frames.
  uint32_t v[9] = {};
  const char *p = line;
  if (strncmp(p, "G2,", 3) != 0) return false;
  p += 3;
  for (uint8_t i = 0; i < 9; ++i) {
    if (*p < '0' || *p > '9') return false;
    while (*p >= '0' && *p <= '9') {
      uint32_t digit = (uint32_t)(*p++ - '0');
      if (v[i] > (UINT32_MAX - digit) / 10U) return false;
      v[i] = v[i] * 10U + digit;
    }
    if (i < 8) { if (*p++ != ',') return false; }
    else if (*p != 0) return false;
  }
  if (v[1] < 1 || v[1] > 4 || v[2] > 1 || v[3] > 1 ||
      v[4] > 262143U || v[5] > 15 || v[6] > 3 ||
      v[7] > 255 || v[8] > 255 || (v[3] && !v[2])) return false;
  GasReading &reading = gasReadings[v[1] - 1];
  if (reading.received && millis() - reading.receivedAtMs <= kGasTimeoutMs) {
    uint32_t delta = v[0] - reading.sequence;
    if (!delta || delta >= 0x80000000U) return false;
  }
  reading.sequence = v[0];
  reading.channel = (uint8_t)v[1];
  reading.moduleOnline = v[2] != 0;
  reading.valid = v[3] != 0 && gasStatusUsable((uint8_t)v[7]) && gasUnit((uint8_t)v[5])[0];
  reading.concentrationRaw = v[4];
  reading.unitCode = (uint8_t)v[5];
  reading.decimals = (uint8_t)v[6];
  reading.status = (uint8_t)v[7];
  reading.gasType = (uint8_t)v[8];
  reading.received = true;
  reading.receivedAtMs = millis();
  return true;
}

void sendTelemetryJson() {
  const uint32_t now = millis();
  bool online = telemetry.received && now - telemetry.receivedAtMs <= kTelemetryTimeoutMs;
  // One bounded static response buffer; avoid large nested stack buffers on ESP.
  static char json[4800];
  char roll[16], pitch[16], distance[16], concentration[24], carJson[200];
  bool carOnline=vehicle.received&&now-vehicle.receivedAtMs<=kTelemetryTimeoutMs;
  snprintf(carJson,sizeof(carJson),"{\"online\":%s,\"mode\":%u,\"state\":%u,\"pwm\":[%d,%d,%d,%d]}",
           carOnline?"true":"false",vehicle.mode,vehicle.state,
           carOnline?vehicle.pwm[0]:0,carOnline?vehicle.pwm[1]:0,
           carOnline?vehicle.pwm[2]:0,carOnline?vehicle.pwm[3]:0);
  formatTenths(roll, sizeof(roll), telemetry.rollTenths);
  formatTenths(pitch, sizeof(pitch), telemetry.pitchTenths);
  if (online && telemetry.distanceValid) formatTenths(distance, sizeof(distance), telemetry.distanceTenths);
  else strcpy(distance, "null");
  int count = snprintf(json, sizeof(json),
    "{\"gasProtocol\":2,\"sequence\":%lu,\"roll\":%s,\"pitch\":%s,\"distance\":%s,\"distanceValid\":%s,\"link\":%s,\"online\":%s,\"vehicle\":%s,\"gases\":[",
    (unsigned long)telemetry.sequence, roll, pitch, distance,
    online && telemetry.distanceValid ? "true" : "false",
    online && telemetry.link ? "true" : "false", online ? "true" : "false", carJson);
  size_t used = count > 0 ? (size_t)count : sizeof(json);
  for (unsigned i = 0; i < 4 && used < sizeof(json); ++i) {
    const GasReading &gas = gasReadings[i];
    bool gasOnline = gas.received && gas.moduleOnline && now - gas.receivedAtMs <= kGasTimeoutMs;
    bool valid = gasOnline && gas.valid;
    formatScaled(concentration, sizeof(concentration), gas.concentrationRaw, gas.decimals);
    const char *statusText = !gasOnline ? "未连接 / 无新数据" :
      gas.status == 1 && !valid ? "单位或精度编码异常" : gasStatusText(gas.status);
    count = snprintf(json + used, sizeof(json) - used,
      "%s{\"channel\":%u,\"name\":\"%s\",\"valid\":%s,\"online\":%s,\"concentration\":%s,\"concentrationRaw\":%lu,\"unitCode\":%u,\"unit\":\"%s\",\"decimals\":%u,\"status\":%u,\"statusText\":\"%s\",\"severity\":\"%s\",\"gasType\":%u,\"temperature\":null,\"humidity\":null}",
      i ? "," : "", i + 1, gas.received ? gasName(gas.gasType) : "待识别气体",
      valid ? "true" : "false", gasOnline ? "true" : "false",
      valid ? concentration : "null", (unsigned long)gas.concentrationRaw,
      gas.unitCode, gasUnit(gas.unitCode), gas.decimals, gas.status, statusText,
      gasOnline ? gasSeverity(gas.status, valid) : "offline", gas.gasType);
    if (count < 0 || (size_t)count >= sizeof(json) - used) { used = sizeof(json); break; }
    used += (size_t)count;
  }
  addCorsHeaders();
  if (used + 3 > sizeof(json)) {
    server.send(500, "application/json", "{\"error\":\"telemetry_overflow\"}");
    return;
  }
  json[used++]=']';json[used]=0;
  used=appendEncoderJson(json,used,sizeof(json),now);
  used=appendWheelSpeedJson(json,used,sizeof(json),now);
  if(used+2>sizeof(json)){
    server.send(500,"application/json","{\"error\":\"telemetry_overflow\"}");return;
  }
  strcpy(json+used,"}");
  server.send(200, "application/json; charset=utf-8", json);
}

void sendOptions() {
  addCorsHeaders();
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type, X-Route-Token");
  server.sendHeader("Access-Control-Max-Age", "600");
  server.send(204, "text/plain", "");
}

bool parseTelemetryLine(const char *line) {
  unsigned long sequence; long roll, pitch, distance; unsigned int valid, link; char trailing;
  int fields = sscanf(line, "T,%lu,%ld,%ld,%ld,%u,%u%c", &sequence, &roll, &pitch, &distance, &valid, &link, &trailing);
  if (fields != 6 || roll < -1800 || roll > 1800 || pitch < -1800 || pitch > 1800 || valid > 1 || link > 1) return false;
  if (valid && (distance < 20 || distance > 4000)) return false;
  telemetry.sequence = (uint32_t)sequence;
  telemetry.rollTenths = (int32_t)roll;
  telemetry.pitchTenths = (int32_t)pitch;
  telemetry.distanceTenths = (int32_t)distance;
  telemetry.distanceValid = valid != 0;
  telemetry.link = link != 0;
  telemetry.received = true;
  telemetry.receivedAtMs = millis();
  return true;
}

bool parseVehicleLine(const char *line) {
  unsigned long seq;unsigned mode,state;int p[4];char trailing;
  int fields=sscanf(line,"V,%lu,%u,%u,%d,%d,%d,%d%c",&seq,&mode,&state,&p[0],&p[1],&p[2],&p[3],&trailing);
  if(fields!=7||mode>5||state>6)return false;
  for(unsigned i=0;i<4;i++)if(p[i]<-1000||p[i]>1000)return false;
  if(vehicle.received&&millis()-vehicle.receivedAtMs<=kTelemetryTimeoutMs) {
    uint32_t delta=(uint32_t)seq-vehicle.sequence;
    if(!delta||delta>=0x80000000U)return false;
  }
  vehicle.sequence=(uint32_t)seq;vehicle.mode=(uint8_t)mode;vehicle.state=(uint8_t)state;
  for(unsigned i=0;i<4;i++)vehicle.pwm[i]=(int16_t)p[i];
  vehicle.received=true;vehicle.receivedAtMs=millis();return true;
}

void readStm32Telemetry() {
  unsigned budget=1024;
  while (budget-- && Serial.available()) {
    char byte = (char)Serial.read();
    if (byte == '\r') continue;
    if (byte == '\n') {
      if (!discardTelemetryLine && telemetryLength) {
        telemetryLine[telemetryLength] = 0;
        if (telemetryLine[0] == 'T') parseTelemetryLine(telemetryLine);
        else if (telemetryLine[0] == 'G') parseGasLine(telemetryLine);
        else if (telemetryLine[0] == 'V') parseVehicleLine(telemetryLine);
        else if (telemetryLine[0] == 'E') parseEncoderLine(telemetryLine);
        else if (telemetryLine[0] == 'S') parseWheelSpeedLine(telemetryLine);
        else if (telemetryLine[0] == 'J') parseJournalLine(telemetryLine);
      }
      telemetryLength = 0; discardTelemetryLine = false; continue;
    }
    if (discardTelemetryLine) continue;
    if (telemetryLength >= kTelemetryLineCapacity - 1) { telemetryLength = 0; discardTelemetryLine = true; continue; }
    telemetryLine[telemetryLength++] = byte;
  }
}

void forwardHandheldUdp() {
  static bool sourceKnown=false, queued=false;
  static IPAddress sourceIp;
  static uint32_t lastSequence=0,lastAt=0;
  static HandheldFrame newest;
  if(sourceKnown&&millis()-lastAt>500){sourceKnown=false;queued=false;}
  if(queued&&millis()-lastAt>100)queued=false; // Never replay stale control after congestion.
  // Drain backlog in bounded batches; forward only the newest validated packet.
  for(unsigned count=0;count<16;count++) {
    int packetSize=udp.parsePacket();
    if(packetSize<=0) {
      if(queued) {
        char line[64];int n=HH_Format(line,sizeof(line),&newest);
        if(n>0&&Serial.availableForWrite()>=n){Serial.write((const uint8_t*)line,n);queued=false;}
      }
      return;
    }
    IPAddress ip=udp.remoteIP();
    int n=udp.read(udpPacket,kUdpPacketCapacity-1);
    if(n<=0||n!=packetSize||n>=64)continue;
    while(n>0&&(udpPacket[n-1]=='\n'||udpPacket[n-1]=='\r'))n--;
    HandheldFrame candidate;
    if(!HH_Parse(udpPacket,n,&candidate))continue;
    if(sourceKnown) {
      if(ip!=sourceIp)continue;
      uint32_t delta=candidate.sequence-lastSequence;
      if(!delta||delta>=0x80000000U)continue;
    }
    sourceKnown=true;sourceIp=ip;lastSequence=candidate.sequence;lastAt=millis();
    newest=candidate;queued=true;
  }
  // If more remain, defer transmission until the next drained batch.
}

void sendControlPage() { server.send_P(200, "text/html; charset=utf-8", kControlPage); }

void sendScanCommand() {
  String mode = server.arg("mode");
  const char *command = 0;
  const char *message = 0;
  if (mode == "horizontal") { command = "M,HORIZONTAL\n"; message = "左右扫描已启动"; }
  else if (mode == "vertical") { command = "M,VERTICAL\n"; message = "上下扫描已启动"; }
  else if (mode == "all") { command = "M,ALL\n"; message = "全方位扫描已启动"; }
  else if (mode == "stop") { command = "M,STOP\n"; message = "停车与停止扫描指令已发送，请核对主控状态"; }
  else if (mode == "handheld") { command = "M,HANDHELD\n"; message = "已恢复体感控制"; }
  else { addCorsHeaders(); server.send(400, "application/json; charset=utf-8", "{\"ok\":false,\"message\":\"无效模式\"}"); return; }
  if(Serial.availableForWrite()<(int)strlen(command)){
    addCorsHeaders();server.send(503,"application/json","{\"ok\":false,\"message\":\"串口忙，请重试\"}");return;
  }
  Serial.write((const uint8_t *)command, strlen(command));
  addCorsHeaders();
  String body = String("{\"ok\":true,\"message\":\"") + message + "\"}";
  server.send(200, "application/json; charset=utf-8", body);
}

}  // namespace

void setup() {
  Serial.setRxBufferSize(1024);
  Serial.begin(115200);
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.softAP(kApName, kApPassword);
  encoderBoot=ESP.getCycleCount();
  journalClient=ESP.random();if(!journalClient)journalClient=1;
  udp.begin(kUdpPort);
  server.on("/", HTTP_GET, sendControlPage);
  server.on("/api/telemetry", HTTP_GET, sendTelemetryJson);
  server.on("/api/telemetry", HTTP_OPTIONS, sendOptions);
  server.on("/api/scan", HTTP_GET, sendScanCommand);
  server.on("/api/scan", HTTP_OPTIONS, sendOptions);
  server.on("/api/route",HTTP_GET,sendRouteRemoved);
  server.on("/api/route",HTTP_POST,sendRouteRemoved);
  server.on("/api/route",HTTP_OPTIONS,sendOptions);
  server.on("/api/route-log",HTTP_GET,sendJournal);
  server.on("/api/route-log",HTTP_OPTIONS,sendOptions);
  server.onNotFound([]() { addCorsHeaders(); server.send(404, "application/json", "{\"error\":\"not_found\"}"); });
  server.begin();
}

void loop() {
  // Latch expiration, including across the millis() wrap.
  const uint32_t now = millis();
  if (telemetry.received && now - telemetry.receivedAtMs > kTelemetryTimeoutMs) telemetry.received = false;
  for (unsigned i = 0; i < 4; ++i)
    if (gasReadings[i].received && now - gasReadings[i].receivedAtMs > kGasTimeoutMs) gasReadings[i].received = false;
  if(vehicle.received&&now-vehicle.receivedAtMs>kTelemetryTimeoutMs)vehicle.received=false;
  if(encoderFresh&&now-encoderAt>kTelemetryTimeoutMs)encoderFresh=false;
  forwardHandheldUdp();
  readStm32Telemetry();
  server.handleClient();
  forwardHandheldUdp();
  readStm32Telemetry();
  yield();

}
