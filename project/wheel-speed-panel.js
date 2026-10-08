/* V8.2 / 2026-09-13: read-only MCU wheel-speed diagnostics, no drive API. */
(function(root){
  'use strict';
  const target=document.getElementById('vehicle-route-panel');if(!target)return;
  const host=document.createElement('section');host.id='wheel-speed-panel';host.style.cssText='display:block;grid-column:1/-1;margin:22px 0 0';target.after(host);
  const shadow=host.attachShadow({mode:'open'});
  shadow.innerHTML=`<style>
  :host{color:#eaf4ee;font:14px/1.6 "Segoe UI","Microsoft YaHei",sans-serif}*{box-sizing:border-box}.card{background:#15251b;border:1px solid #35523e;border-radius:12px;overflow:hidden}header{padding:18px 22px;border-bottom:1px solid #304b39;display:flex;justify-content:space-between;gap:14px}h2{margin:0;font-size:19px}p{margin:3px 0;color:#96b19f;font-size:12px}.tag{font-size:12px;color:#e6c789;align-self:center}.wheels{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px;padding:18px 22px}.wheel{padding:13px;border:1px solid #304b39;border-radius:8px;background:#102117}.name{color:#a5bdae;font-size:12px}.rpm{font-size:27px;color:#77dfa4;font-variant-numeric:tabular-nums}.row{display:flex;justify-content:space-between;gap:8px;font-size:12px;color:#9fb6a7}.row b{color:#e4efe8;font-weight:500}.bar{height:5px;background:#263f2f;border-radius:4px;overflow:hidden;margin:10px 0}.bar i{display:block;height:100%;width:0;background:#70d8a0;transition:width .15s}.note{border-top:1px solid #304b39;padding:12px 22px;font-size:12px;color:#b2caba;overflow-wrap:anywhere}.error{color:#ffc785}canvas{display:block;width:100%;height:135px;padding:0 22px}button{border:1px solid #3a5843;border-radius:6px;padding:6px 10px;color:#d9eee1;background:#213a2a;cursor:pointer;font:inherit;font-size:12px}button:disabled{opacity:.4;cursor:not-allowed}.toolbar{display:flex;gap:12px;align-items:center;padding:0 22px 12px;color:#9fb6a7;font-size:12px}@media(max-width:760px){header{flex-wrap:wrap}.wheels{grid-template-columns:1fr 1fr;padding:14px;gap:9px}.rpm{font-size:23px}.note{padding:12px 14px}}
  </style><div class="card"><header><div><h2>四轮速度反馈 <small>V8.2</small></h2><p>为低速路线复走准备 · 本面板只读，不会启动小车</p></div><span id="state" class="tag">等待新版主控诊断</span></header><div class="wheels" id="wheels"></div><canvas id="chart" aria-label="四轮实测转速曲线"></canvas><div class="toolbar"><button id="export" disabled>导出轮速诊断</button><span>最近约24秒 · 1绿 / 2蓝 / 3黄 / 4紫</span></div><div class="note" id="note">需要 STM32 V8.2 和基座 ESP V4.8。原有遥控、轨迹和档案功能不受影响。</div></div>`;
  const $=id=>shadow.getElementById(id),names=['1 左前','2 右前','3 左后','4 右后'];
  names.forEach((name,i)=>{const card=document.createElement('div');card.className='wheel';card.innerHTML=`<div class="name">${name}</div><div class="rpm"><span id="rpm${i}">--</span> <small style="font-size:11px">RPM</small></div><div class="row"><span>目标</span><b id="target${i}">--</b></div><div class="bar"><i id="bar${i}"></i></div><div class="row"><span>PWM 命令</span><b id="pwm${i}">--</b></div>`;$('wheels').append(card);});
  let data=null,lastAt=0,lastSequence=null,history=[];
  const array=(a,min,max)=>Array.isArray(a)&&a.length===4&&a.every(v=>Number.isInteger(v)&&v>=min&&v<=max);
  function valid(d){return d&&d.protocol===1&&typeof d.online==='boolean'&&typeof d.valid==='boolean'&&typeof d.enabled==='boolean'&&Number.isInteger(d.sequence)&&d.sequence>=0&&d.sequence<=4294967295&&Number.isFinite(d.ageMs)&&d.ageMs>=0&&d.ageMs<=1000&&array(d.targetRpm10,-1000,1000)&&array(d.measuredRpm10,-20000,20000)&&array(d.pwm,-300,300)&&array(d.signs,-1,1)&&d.signs.every(n=>n!==0)&&Number.isInteger(d.cpr)&&d.cpr>=1&&d.cpr<=1000000&&Number.isInteger(d.fault)&&d.fault>=0&&d.fault<=15&&Number.isInteger(d.faultWheels)&&d.faultWheels>=0&&d.faultWheels<=15&&Number.isInteger(d.limited)&&d.limited>=0&&d.limited<=15&&Number.isInteger(d.sampleMs)&&d.sampleMs>=0&&d.sampleMs<=65535;}
  function draw(){
    const canvas=$('chart'),r=canvas.getBoundingClientRect(),w=Math.max(1,r.width-44),h=135,ratio=Math.min(devicePixelRatio||1,2),ctx=canvas.getContext('2d');canvas.width=w*ratio;canvas.height=h*ratio;ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,w,h);
    const max=Math.max(30,...history.flatMap(p=>p.rpm.map(Math.abs))),left=35,right=w-6,top=14,bottom=h-19,mid=(top+bottom)/2;
    ctx.font='10px Segoe UI';[-max,0,max].forEach(v=>{const y=mid-v/max*(bottom-top)/2;ctx.strokeStyle='#294333';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();ctx.fillStyle='#8ba696';ctx.fillText(v.toFixed(0),1,y+3);});
    ['#74dca4','#7fbdff','#ecc56b','#c79cf7'].forEach((color,j)=>{ctx.strokeStyle=color;ctx.lineWidth=1.5;ctx.beginPath();history.forEach((p,i)=>{const x=left+(right-left)*i/119,y=mid-p.rpm[j]/max*(bottom-top)/2;if(!i||p.gap)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.stroke();});
    ctx.fillStyle='#8ba696';ctx.fillText('RPM',1,10);
  }
  function render(){
    const live=data&&data.online&&Date.now()-lastAt+data.ageMs<=1000,usable=live&&data.valid;
    $('state').textContent=!live?'轮速诊断离线':data.fault?(data.enabled?'反馈异常 · 闭环已停车':'上次自动行驶故障 · 已保留'):data.enabled?'闭环模式':'监测模式 · 未接管电机';
    names.forEach((_,i)=>{
      $('rpm'+i).textContent=usable?(data.measuredRpm10[i]/10).toFixed(1):'--';
      $('target'+i).textContent=live&&data.enabled?(data.targetRpm10[i]/10).toFixed(1)+' RPM':'--';
      $('pwm'+i).textContent=live?(data.pwm[i]/10).toFixed(1)+'%':'--';
      $('bar'+i).style.width=live?Math.abs(data.pwm[i])/3+'%':'0%';
    });
    if(!live){$('note').textContent='需要 STM32 V8.2 和基座 ESP V4.8 的新数据。数值未知不显示成0；原有遥控不由本面板控制。';return;}
    let note=data.enabled?'主控根据编码器调节轮速，仍需实测调参；轮速闭环不能消除轮胎打滑。':'当前只监测轮速，电机继续原手持开环控制。目标转速不适用，因此显示横线。';
    note+=' 主控：'+data.cpr+'次/圈，轮向 '+data.signs.map(n=>n>0?'+1':'−1').join('/')+(data.fault?'，故障采样间隔 ':'，采样间隔 ')+data.sampleMs+'ms。';
    const c=root.VehicleRoute&&root.VehicleRoute.engine.config;
    if(c&&c.confirmed&&(c.cpr!==data.cpr||c.signs.some((n,i)=>n!==data.signs[i])))note+=' 注意：网页轨迹标定与主控轮速参数不一致；修改网页不会改变主控配置。';
    if(!data.valid)note+=' 本次反馈采样无效。';
    if(data.fault){const causes=[];if(data.fault&1)causes.push('采样超时');if(data.fault&2)causes.push('计数跳变');if(data.fault&4)causes.push('反馈反向');if(data.fault&8)causes.push('持续无脉冲/可能堵转');note+=' 异常：'+causes.join('、')+'，涉及轮子 '+names.filter((_,i)=>data.faultWheels&(1<<i)).join(' / ')+'。故障记录保留至下次路线上传；请先排查再重试。';}
    if(data.limited)note+=' 部分轮子达到30% PWM上限，不保证已经达到目标转速。';
    $('note').textContent=note;$('note').className='note'+(data.fault?' error':'');
  }
  root.WheelSpeedPanel={feed(d){
    if(!valid(d)){data=null;render();return;}
    data=d;lastAt=Date.now();
    if(d.online&&d.valid&&d.sequence!==lastSequence){
      const last=history[history.length-1];history.push({at:lastAt,rpm:d.measuredRpm10.map(n=>n/10),target:d.enabled?d.targetRpm10.map(n=>n/10):null,pwm:d.pwm.slice(),enabled:d.enabled,fault:d.fault,faultWheels:d.faultWheels,sampleMs:d.sampleMs,gap:!!last&&(lastAt-last.at>700||((d.sequence-lastSequence)>>>0)>=2147483648)});
      if(history.length>120)history.shift();lastSequence=d.sequence;$('export').disabled=false;draw();
    }
    render();
  }};
  $('export').onclick=()=>{
    const lines=['# 轮速诊断；PWM为逻辑命令，不是电流或驱动执行确认。','time,closed_loop,fault,fault_wheels,sample_ms,rpm1,rpm2,rpm3,rpm4,target1,target2,target3,target4,pwm1,pwm2,pwm3,pwm4',...history.map(p=>[new Date(p.at).toISOString(),p.enabled?1:0,p.fault,p.faultWheels,p.sampleMs,...p.rpm,...(p.target||['','','','']),...p.pwm].join(','))];
    const url=URL.createObjectURL(new Blob(['\uFEFF'+lines.join('\r\n')],{type:'text/csv;charset=utf-8'})),a=document.createElement('a');a.href=url;a.download='四轮诊断-'+Date.now()+'.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  };
  new ResizeObserver(draw).observe($('chart'));setInterval(render,500);draw();
})(window);
