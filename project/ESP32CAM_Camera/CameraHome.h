#pragma once
// V1.2 / V8.5：10/5/3帧选项，上下翻转已由摄像头完成。默认关闭，不控制小车。
static const char CAMERA_HOME[] PROGMEM = R"CAMERAHTML(<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>嵌入电团队 · 车载画面</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#101814;color:#edf5ef;font:15px/1.6 system-ui,"Microsoft YaHei",sans-serif}
main{max-width:780px;margin:auto;padding:24px}h1{font-size:22px;margin-bottom:2px}p{color:#9eb6a7}
button{font:inherit;padding:10px 18px;border:1px solid #395548;border-radius:8px;background:#22382c;color:inherit;cursor:pointer}
select{font:inherit;padding:10px;border:1px solid #395548;border-radius:8px;background:#22382c;color:inherit}label{display:inline-block;margin-top:8px}
button:first-child{background:#54da9d;color:#102419;font-weight:600}.view{margin:18px 0;background:#080f0b;aspect-ratio:4/3;border:1px solid #395548;border-radius:12px;display:grid;place-items:center;position:relative;overflow:hidden}
img{width:100%;height:100%;object-fit:contain}img[hidden]{display:none}#empty{position:absolute;padding:20px;text-align:center;color:#a7b8af}#status{min-height:48px}
small{color:#9eb6a7}a{color:#54da9d}</style>
<main><h1>嵌入电团队 · 车载画面</h1><p>320 × 240 · 最高约 10 帧/秒 · 摄像头 V1.2 · ESP32-CAM</p>
<div><button id="start">开启画面</button> <button id="stop" disabled>关闭画面</button> <label>帧率 <select id="rate" aria-label="视频帧率"><option value="10">10 帧/秒</option><option value="5">5 帧/秒</option><option value="3">3 帧/秒</option></select></label></div>
<div class="view"><img id="frame" alt="小车前方摄像头画面" hidden><span id="empty">画面未开启</span></div>
<div id="status" role="status">只显示摄像头，不发送车辆控制指令。</div>
<small>只在一个窗口开启视频。遥控变卡先停车，再降至5或3帧。切到其他页面会关闭收图。画面中断请立即停止驾驶；画面延迟不等于遥控链路延迟。</small></main>
<script>
const el=id=>document.getElementById(id);
let active=false,generation=0,controller=null,timer=0,objectUrl='',lastId='',lastAt=0;
function clearFrame(text){el('frame').hidden=true;el('frame').removeAttribute('src');if(objectUrl)URL.revokeObjectURL(objectUrl);objectUrl='';el('empty').hidden=false;el('empty').textContent=text;}
function stop(text='画面已关闭'){active=false;generation++;clearTimeout(timer);if(controller)controller.abort();controller=null;clearFrame(text);el('status').textContent=text;el('start').disabled=false;el('stop').disabled=true;el('rate').disabled=false;}
async function poll(g,viewer){
 if(!active||g!==generation)return;
 const began=performance.now(),abort=new AbortController();controller=abort;const deadline=setTimeout(()=>abort.abort(),1200);
 let pending='';
 try{
  const r=await fetch('/capture?viewer='+viewer+'&t='+Date.now(),{cache:'no-store',signal:abort.signal});
  if(r.status===429){if(active&&g===generation)timer=setTimeout(()=>poll(g,viewer),250);return;}
  if(r.status===409)throw Error('已有其他窗口收图，请先关闭它，等3秒再试。');
  if(!r.ok)throw Error('摄像头响应 '+r.status+'，请查看串口日志或 /status。');
  if(r.headers.get('X-Camera-Version')!=='1.0')throw Error('摄像头固件版本不匹配');
  const id=r.headers.get('X-Frame-Id'),ageText=r.headers.get('X-Frame-Age-Ms'),age=ageText===null?NaN:Number(ageText);
  if(!id||id===lastId||!Number.isFinite(age)||age<0||age>500)throw Error('未收到新鲜画面');
  const blob=await r.blob();if(blob.type!=='image/jpeg'||blob.size<100||blob.size>160000)throw Error('图像格式异常');
  pending=URL.createObjectURL(blob);const probe=new Image();probe.src=pending;await probe.decode();
  if(!active||g!==generation)return;
  if(abort.signal.aborted)throw Error('收图超时');
  if(performance.now()-began>1000)throw Error('收图耗时过长');
  if(objectUrl)URL.revokeObjectURL(objectUrl);objectUrl=pending;pending='';
  el('frame').src=objectUrl;el('frame').hidden=false;el('empty').hidden=true;lastId=id;lastAt=performance.now();
  el('status').textContent='持续收图 · 本次收图 '+Math.round(lastAt-began)+' ms（不是端到端延迟）';
  timer=setTimeout(()=>poll(g,viewer),Math.max(Math.max(105,1000/Number(el('rate').value))-(performance.now()-began),0));
 }catch(e){if(g===generation)stop('画面中断，先停止驾驶。'+e.message);}
 finally{clearTimeout(deadline);if(pending)URL.revokeObjectURL(pending);if(controller===abort)controller=null;}
}
el('start').onclick=()=>{stop();active=true;lastId='';lastAt=0;el('start').disabled=true;el('stop').disabled=false;el('rate').disabled=true;el('status').textContent='正在连接…';clearFrame('等待新画面');const viewer=Array.from(crypto.getRandomValues(new Uint8Array(16)),b=>b.toString(16).padStart(2,'0')).join('');poll(generation,viewer);};
el('stop').onclick=()=>stop();
setInterval(()=>{if(active&&lastAt&&performance.now()-lastAt>1000){clearFrame('画面过期，请停止驾驶');el('status').textContent='画面过期，等待新帧';}},200);
document.addEventListener('visibilitychange',()=>{if(document.hidden)stop('页面已隐藏，画面关闭；返回后请重新开启。');});
window.addEventListener('pagehide',()=>stop());
</script></html>)CAMERAHTML";
