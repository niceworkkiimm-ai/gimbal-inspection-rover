#include "secrets.h"
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char *AP_SSID = "Gimbal-8266";
const char *AP_PASSWORD = GIMBAL_WIFI_PASSWORD;
const uint16_t HTTP_PORT = 80;
const uint32_t MCU_BAUD_RATE = 115200;

ESP8266WebServer server(HTTP_PORT);
String mcuLine;
String lastStatus = "STATUS,0,0,-1";
uint32_t lastStatusTime;

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>二维云台控制</title>
  <style>
    *{box-sizing:border-box}body{margin:0;background:#f4f6f5;color:#17201c;font-family:Arial,"Microsoft YaHei",sans-serif}
    main{max-width:520px;margin:auto;padding:22px 18px}.top{display:flex;align-items:center;justify-content:space-between;margin-bottom:20px}
    h1{font-size:22px;margin:0}.state{font-size:13px;color:#52615a}.values{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:22px}
    .value{background:#fff;border:1px solid #dce3df;border-radius:6px;padding:14px 8px;text-align:center}.value b{display:block;font-size:22px;margin-top:5px}
    .controls{display:grid;gap:10px}button{min-height:50px;border:0;border-radius:6px;font-size:16px;font-weight:600;color:#fff;background:#18794e}
    button.all{background:#2864a8}button.stop{background:#b33a3a}button:active{opacity:.76}.note{margin-top:18px;color:#617069;font-size:13px;text-align:center}
  </style>
</head>
<body><main>
  <div class="top"><h1>二维云台控制</h1><span id="state" class="state">等待设备</span></div>
  <section class="values">
    <div class="value">水平角<b id="roll">0</b></div>
    <div class="value">俯仰角<b id="pitch">0</b></div>
    <div class="value">距离 cm<b id="distance">--</b></div>
  </section>
  <section class="controls">
    <button onclick="scan('horizontal')">左右扫描</button>
    <button onclick="scan('vertical')">上下扫描</button>
    <button class="all" onclick="scan('all')">全范围扫描</button>
    <button class="stop" onclick="scan('stop')">停止并回中</button>
  </section>
  <div id="message" class="note">连接 Gimbal-8266 后使用</div>
</main>
<script>
async function scan(mode){
  try{const r=await fetch('/api/scan?mode='+mode);const j=await r.json();document.getElementById('message').textContent=j.message||'指令已发送'}
  catch(e){document.getElementById('message').textContent='发送失败'}
}
async function update(){
  try{const r=await fetch('/api/status');const j=await r.json();
    document.getElementById('roll').textContent=j.roll;document.getElementById('pitch').textContent=j.pitch;
    document.getElementById('distance').textContent=j.distance<0?'--':j.distance;
    document.getElementById('state').textContent=j.online?'STM32 在线':'等待 STM32';}
  catch(e){document.getElementById('state').textContent='ESP 连接失败'}
}
setInterval(update,1000);update();
</script></body></html>
)HTML";

static void SendJson(int code, const String &body)
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(code, "application/json; charset=utf-8", body);
}

static void HandleScan(void)
{
  String mode = server.arg("mode");
  String command;
  String label;

  if (mode == "horizontal") { command = "SCAN,HORIZONTAL"; label = "左右扫描已启动"; }
  else if (mode == "vertical") { command = "SCAN,VERTICAL"; label = "上下扫描已启动"; }
  else if (mode == "all") { command = "SCAN,ALL"; label = "全范围扫描已启动"; }
  else if (mode == "stop") { command = "SCAN,STOP"; label = "扫描已停止"; }
  else { SendJson(400, "{\"ok\":false,\"message\":\"无效模式\"}"); return; }

  Serial.println(command);
  SendJson(200, String("{\"ok\":true,\"message\":\"") + label + "\"}");
}

static void HandleControl(void)
{
  int roll = server.arg("roll").toInt();
  int pitch = server.arg("pitch").toInt();

  if (roll < -90 || roll > 90 || pitch < -90 || pitch > 90)
  {
    SendJson(400, "{\"ok\":false,\"message\":\"角度超出范围\"}");
    return;
  }

  Serial.printf("CTRL,%d,%d\n", roll, pitch);
  SendJson(200, "{\"ok\":true,\"message\":\"角度已更新\"}");
}

static void HandleStatus(void)
{
  int roll = 0;
  int pitch = 0;
  int distance = -1;
  bool online = (millis() - lastStatusTime) < 2000;

  sscanf(lastStatus.c_str(), "STATUS,%d,%d,%d", &roll, &pitch, &distance);
  SendJson(200, String("{\"online\":") + (online ? "true" : "false") +
                ",\"roll\":" + roll + ",\"pitch\":" + pitch +
                ",\"distance\":" + distance + "}");
}

static void ReadMcuStatus(void)
{
  while (Serial.available())
  {
    char byte = (char)Serial.read();
    if (byte == '\r') continue;

    if (byte == '\n')
    {
      if (mcuLine.startsWith("STATUS,"))
      {
        lastStatus = mcuLine;
        lastStatusTime = millis();
      }
      mcuLine = "";
    }
    else if (mcuLine.length() < 95)
    {
      mcuLine += byte;
    }
    else
    {
      mcuLine = "";
    }
  }
}

void setup()
{
  bool apStarted;

  Serial.begin(MCU_BAUD_RATE);
  Serial.setDebugOutput(false);
  mcuLine.reserve(96);
  lastStatus.reserve(96);

  WiFi.mode(WIFI_AP);
  apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD, 6, false, 4);

  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html; charset=utf-8", INDEX_HTML); });
  server.on("/api/scan", HTTP_GET, HandleScan);
  server.on("/api/control", HTTP_GET, HandleControl);
  server.on("/api/status", HTTP_GET, HandleStatus);
  server.onNotFound([]() { SendJson(404, "{\"ok\":false,\"message\":\"接口不存在\"}"); });
  server.begin();

  if (apStarted)
  {
    Serial.print("AP,OK,");
    Serial.print(WiFi.softAPIP());
    Serial.print(",PORT,");
    Serial.println(HTTP_PORT);
  }
  else
  {
    Serial.println("AP,FAIL");
  }
}

void loop()
{
  server.handleClient();
  ReadMcuStatus();
  yield();
}
