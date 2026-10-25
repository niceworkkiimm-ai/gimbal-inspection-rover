/* V8.14 / 2026-10-25. Deterministic, offline presentation data; never controls hardware. */
(function (root) {
  'use strict';
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const rad = a => a * Math.PI / 180;
  const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
  const mix = (a, b, u) => a + (b - a) * u;
  const SOURCE = 'simulation';
  const STATIONS = [
    {name:'双侧通道',description:'左右墙面与前方通道',position:[1.8,.05]},
    {name:'转角墙面',description:'前墙与右侧墙形成转角',position:[3.2,1.1]},
    {name:'台阶障碍',description:'逐级升高的台阶和低矮障碍',position:[3.45,3.1]},
    {name:'碎石区域',description:'高低不一的碎石与远处墙面',position:[1.4,4.15]},
    {name:'柱体通道',description:'圆柱障碍与侧面墙体',position:[-.4,2.45]}
  ];
  const CONFIG = {diameter:97,cpr:374,track:202,wheelbase:210,signs:[1,-1,1,-1],confirmed:true};
  const WAYPOINTS = [[0,0],...STATIONS.map(s=>s.position),[0,0]];
  function curve(segment,u) {
    const p0=WAYPOINTS[Math.max(0,segment-1)],p1=WAYPOINTS[segment],
      p2=WAYPOINTS[segment+1],p3=WAYPOINTS[Math.min(WAYPOINTS.length-1,segment+2)];
    const values=[0,1].map(k=>.5*((2*p1[k])+(-p0[k]+p2[k])*u+
      (2*p0[k]-5*p1[k]+4*p2[k]-p3[k])*u*u+(-p0[k]+3*p1[k]-3*p2[k]+p3[k])*u*u*u));
    const derivative=[0,1].map(k=>.5*((-p0[k]+p2[k])+
      2*(2*p0[k]-5*p1[k]+4*p2[k]-p3[k])*u+3*(-p0[k]+3*p1[k]-3*p2[k]+p3[k])*u*u));
    return {x:values[0],y:values[1],heading:Math.atan2(derivative[1],derivative[0])};
  }
  // Ray intersections in centimetres: x right, y above the sensor, z forward.
  function boxHit(o,d,b) {
    let lo=0,hi=150;
    for(let i=0;i<3;i++) {
      const low=b.center[i]-b.size[i]/2,high=b.center[i]+b.size[i]/2;
      if(Math.abs(d[i])<1e-9){if(o[i]<low||o[i]>high)return Infinity;continue;}
      let a=(low-o[i])/d[i],c=(high-o[i])/d[i];if(a>c)[a,c]=[c,a];
      lo=Math.max(lo,a);hi=Math.min(hi,c);if(lo>hi)return Infinity;
    }
    return lo>=2?lo:hi>=2?hi:Infinity;
  }
  function ellipsoidHit(o,d,b) {
    const p=o.map((v,i)=>(v-b.center[i])/b.radius[i]),v=d.map((n,i)=>n/b.radius[i]);
    const a=v.reduce((s,n)=>s+n*n,0),bb=2*p.reduce((s,n,i)=>s+n*v[i],0),c=p.reduce((s,n)=>s+n*n,0)-1;
    const q=bb*bb-4*a*c;if(q<0)return Infinity;
    const t=(-bb-Math.sqrt(q))/(2*a);return t>=2?t:Infinity;
  }
  function cylinderHit(o,d,b) {
    const x=o[0]-b.x,z=o[2]-b.z,a=d[0]*d[0]+d[2]*d[2],bb=2*(x*d[0]+z*d[2]);
    let nearest=Infinity;
    const q=bb*bb-4*a*(x*x+z*z-b.radius*b.radius);
    if(a>1e-9&&q>=0)for(const sign of [-1,1]){
      const t=(-bb+sign*Math.sqrt(q))/(2*a),y=o[1]+t*d[1];
      if(t>=2&&y>=b.bottom&&y<=b.top)nearest=Math.min(nearest,t);
    }
    if(Math.abs(d[1])>1e-9)for(const y of [b.bottom,b.top]){
      const t=(y-o[1])/d[1];if(t>=2&&(x+t*d[0])**2+(z+t*d[2])**2<=b.radius*b.radius)nearest=Math.min(nearest,t);
    }
    return nearest;
  }
  function terrain(index) {
    const box=(center,size)=>({type:'box',center,size}),wall=box([0,20,137],[240,110,7]);
    const floor=box([0,-32,75],[320,4,310]);
    if(index===0)return [floor,box([-43,10,76],[7,90,160]),box([49,10,76],[7,90,160]),wall];
    if(index===1)return [floor,box([0,18,82],[170,105,7]),box([48,12,44],[7,95,85])];
    if(index===2)return [floor,wall,box([0,-23,40],[94,14,28]),box([0,-15,68],[94,30,28]),box([0,-6,96],[94,48,28]),box([-62,-11,87],[18,38,25])];
    if(index===3)return [floor,wall,
      {type:'ellipsoid',center:[-30,-20,43],radius:[19,13,19]},
      {type:'ellipsoid',center:[-4,-14,61],radius:[20,20,19]},
      {type:'ellipsoid',center:[28,-22,39],radius:[14,11,16]},
      {type:'ellipsoid',center:[34,-12,82],radius:[23,22,22]},
      {type:'ellipsoid',center:[-42,-10,98],radius:[25,24,22]},
      {type:'ellipsoid',center:[0,-21,108],radius:[18,13,20]}];
    return [floor,wall,box([-69,14,78],[6,95,165]),
      {type:'cylinder',x:-26,z:51,radius:11,bottom:-30,top:54},
      {type:'cylinder',x:34,z:85,radius:15,bottom:-30,top:70}];
  }
  function makeScan(index) {
    const objects=terrain(index),points=[];
    for(let row=0;row<13;row++){
      const pitch=-30+row*5,ys=Array.from({length:57},(_,i)=>-70+i*2.5);
      if(row%2)ys.reverse();
      for(const yaw of ys){
        const p=rad(pitch),a=rad(yaw),d=[Math.cos(p)*Math.sin(a),Math.sin(p),Math.cos(p)*Math.cos(a)],
          o=[-6.5*Math.sin(p)*Math.sin(a),6.5*(Math.cos(p)-1),-6.5*Math.sin(p)*Math.cos(a)];
        let distance=Infinity;
        for(const b of objects)distance=Math.min(distance,b.type==='box'?boxHit(o,d,b):b.type==='cylinder'?cylinderHit(o,d,b):ellipsoidHit(o,d,b));
        if(distance>=2&&distance<149.5){
          distance=clamp(distance+.13*Math.sin(yaw*.7+pitch*.9+index*3),2,149.5);
          points.push({distance:Number(distance.toFixed(3)),yaw,pitch});
        }
      }
    }
    return {kind:'3d',dataSource:SOURCE,points,view:{camera:[460,330,600],target:[0,95,150]}};
  }
  function makeScenario() {
    const scans=STATIONS.map((_,i)=>makeScan(i)),phases=[];
    let time=2;
    phases.push({type:'ready',start:0,end:time,segment:0});
    const travelTimes=[11,12,13,15,14,14];
    for(let i=0;i<6;i++){
      phases.push({type:'drive',start:time,end:time+travelTimes[i],segment:i});time+=travelTimes[i];
      if(i<5){phases.push({type:'scan',start:time,end:time+10,segment:i,station:i});time+=10;}
    }
    const duration=time+2;phases.push({type:'done',start:time,end:duration,segment:5});
    const phaseAt=t=>phases.find(p=>t>=p.start&&t<p.end)||phases[phases.length-1];
    function poseAt(t){
      const phase=phaseAt(clamp(t,0,duration));
      if(phase.type==='ready')return curve(0,0);
      if(phase.type!=='drive')return curve(phase.segment,1);
      const v=clamp((t-phase.start)/(phase.end-phase.start),0,1);
      // Ease into/out of each stop, while retaining the same smooth geometric path.
      const u=v*v*(3-2*v);return curve(phase.segment,u);
    }
    const samples=[];let distance=0,previous=poseAt(0),travel=[0,0,0,0];
    const step=.08,k=(CONFIG.track+CONFIG.wheelbase)/2000,tickM=Math.PI*CONFIG.diameter/(1000*CONFIG.cpr);
    for(let n=0;n<=Math.ceil(duration/step);n++){
      const t=Math.min(duration,n*step),pose=poseAt(t),phase=phaseAt(t),dt=n?Math.max(.00001,t-samples[n-1].time):step;
      const length=n?Math.hypot(pose.x-previous.x,pose.y-previous.y):0;
      distance+=length;
      const da=n?wrap(pose.heading-previous.heading):0;
      const wheel=[length-k*da,length+k*da,length-k*da,length+k*da];
      wheel.forEach((value,i)=>travel[i]+=value);
      const rpm=wheel.map(value=>value/dt/(Math.PI*CONFIG.diameter/1000)*60);
      samples.push({...pose,time:t,distance,phase:phase.type,station:phase.station,
        phaseProgress:clamp((t-phase.start)/(phase.end-phase.start),0,1),
        speed:phase.type==='drive'?length/dt:0,rpm:phase.type==='drive'?rpm:[0,0,0,0],
        raw:travel.map((value,i)=>Math.round(value/tickM)*CONFIG.signs[i])});
      previous=pose;
    }
    const checkpoints=phases.filter(p=>p.type==='scan').map((p,i)=>{
      const s=samples[Math.min(samples.length-1,Math.floor(p.end/step))],pose=curve(p.segment,1);
      return {id:i+1,label:STATIONS[i].name,at:p.end,pose:{...pose,time:p.end*1000,distance:s.distance},scan:scans[i]};
    });
    const bounds={minX:Math.min(...samples.map(s=>s.x)),maxX:Math.max(...samples.map(s=>s.x)),
      minY:Math.min(...samples.map(s=>s.y)),maxY:Math.max(...samples.map(s=>s.y))};
    function sampleAt(t){return samples[Math.min(samples.length-1,Math.floor(clamp(t,0,duration)/step))];}
    function routeAt(t,origin=Date.now()){
      t=clamp(t,0,duration);const s=sampleAt(t),elapsed=s.time*1000;
      return {x:s.x,y:s.y,heading:s.heading,distance:s.distance,elapsed,originTime:origin,
        session:'simulation-offline-v1',gaps:0,restarts:0,decimations:0,
        config:{...CONFIG,signs:CONFIG.signs.slice()},dataSource:SOURCE,
        points:samples.filter(p=>p.time<=s.time).map((p,i)=>({x:p.x,y:p.y,heading:p.heading,time:p.time*1000,distance:p.distance,gap:i===0})),
        scans:checkpoints.filter(p=>p.at*1000<=elapsed+.001).map(p=>({id:p.id,label:p.label,
          capturedAt:origin+p.at*1000,pose:{...p.pose},scan:JSON.parse(JSON.stringify(p.scan))})),
        online:true,lastReceived:Date.now(),speed:s.speed,rpm:s.rpm.slice(),raw:s.raw.slice()};
    }
    return {duration,samples,scans,phases,checkpoints,bounds,sampleAt,routeAt};
  }
  const api={makeScenario,makeScan,STATIONS,CONFIG,SOURCE};
  if(typeof module==='object'&&module.exports)module.exports=api;
  root.RoverDemoCore=api;
  if(typeof document==='undefined')return;
  const enabled=new URLSearchParams(root.location.search).get('demo')==='1';
  const demo={enabled};root.RoverDemo=demo;
  if(!enabled)return;
  const scenario=makeScenario(),origin=Date.now();
  Object.assign(demo,{scenario,time:0,playing:false,rate:1,epoch:0,origin,route:scenario.routeAt(0,origin)});
  let bridge=null,previousNow=0,lastPaint=-1,lastTerrain=-1,visiblePoint=-1,ui=null,initialized=false;
  function seek(seconds){demo.time=clamp(seconds,0,scenario.duration);demo.epoch++;lastTerrain=-1;lastPaint=-1;paint(true);}
  function paint(force){
    if(!bridge||!initialized)return;
    const s=scenario.sampleAt(demo.time);
    if(!force&&Math.abs(demo.time-lastPaint)<.075)return;
    lastPaint=demo.time;
    Object.assign(demo.route,scenario.routeAt(demo.time,origin));
    const completed=scenario.phases.filter(p=>p.type==='scan'&&p.end<=demo.time).length;
    const terrainIndex=s.station===undefined?clamp(completed-1,0,4):s.station;
    const scanning=s.phase==='scan',cloudKey=terrainIndex+':'+(scanning?'scan':'hold');
    const newTerrain=lastTerrain!==cloudKey;
    if(newTerrain){bridge.clear();lastTerrain=cloudKey;visiblePoint=-1;}
    const scan=scenario.scans[terrainIndex];
    const count=scanning?Math.max(1,Math.floor(s.phaseProgress*scan.points.length)):completed?scan.points.length:0;
    if(count){
      if(force&&!newTerrain)bridge.clear();
      const start=force||newTerrain?0:visiblePoint+1;
      for(let i=start;i<count;i++)bridge.point(scan.points[i]);
      visiblePoint=count-1;
    }
    const point=scanning?scan.points[Math.max(0,count-1)]:scan.points[Math.floor(demo.time*4)%scan.points.length];
    const sequence=Math.floor(demo.time*1000)+demo.epoch*1000000;
    const gasTime=demo.time*.12;
    const gases=[{name:'甲烷 CH₄',unit:'%LEL',concentration:1.2+.25*Math.sin(gasTime)},
      {name:'氧气 O₂',unit:'%VOL',concentration:20.8+.06*Math.sin(gasTime*.8)},
      {name:'一氧化碳 CO',unit:'ppm',concentration:2.1+.4*Math.sin(gasTime*.6)},
      {name:'硫化氢 H₂S',unit:'ppm',concentration:.15+.03*Math.sin(gasTime)}]
      .map((g,i)=>({...g,channel:i+1,online:true,valid:true,statusText:'模拟 · 正常',severity:'normal'}));
    bridge.frame({sequence,online:true,link:true,roll:point.yaw,pitch:-point.pitch,distance:point.distance,
      distanceValid:true,ageMs:0,gases},false);
    const rpm=s.rpm.map(n=>Math.round(n*10));
    root.WheelSpeedPanel.feed({protocol:1,online:true,valid:true,enabled:false,sequence,ageMs:0,
      targetRpm10:[0,0,0,0],measuredRpm10:rpm,pwm:s.rpm.map(n=>Math.round(clamp(n*3,-300,300))),
      signs:CONFIG.signs,cpr:CONFIG.cpr,fault:0,faultWheels:0,limited:0,sampleMs:80});
    root.VehicleRoute.presentDemo(demo.route,force?scenario.bounds:null);
    document.getElementById('streamText').textContent='模拟数据 · 离线演示';
    document.getElementById('echoText').textContent='模拟测距 · 非现场采集';
    document.getElementById('update').textContent='模拟时间 '+format(demo.time)+' / '+format(scenario.duration);
    const phaseText=s.phase==='ready'?'准备出发':s.phase==='drive'?'行驶中':s.phase==='scan'?'停车扫描 · '+STATIONS[s.station].name:'巡检完成 · 点击路径点回看';
    ui.querySelector('#demo-phase').textContent=phaseText;
    ui.querySelector('#demo-progress').value=String(demo.time);
    ui.querySelector('#demo-clock').textContent=format(demo.time)+' / '+format(scenario.duration);
    ui.querySelector('#demo-play').textContent=demo.playing?'暂停演示':demo.time>=scenario.duration?'重新播放':'开始演示';
    ui.querySelectorAll('[data-station]').forEach((button,i)=>{
      const done=demo.route.scans.some(s=>s.id===i+1);button.classList.toggle('complete',done);
      button.classList.toggle('current',scanning&&s.station===i);button.setAttribute('aria-label',`扫描点 ${i+1} ${STATIONS[i].name}，跳转并查看模拟点图`);
      button.querySelector('small').textContent=scanning&&s.station===i?Math.round(s.phaseProgress*100)+'%':done?'已保存点图':'待扫描';
    });
  }
  const format=t=>`${String(Math.floor(t/60)).padStart(2,'0')}:${String(Math.floor(t%60)).padStart(2,'0')}`;
  function tick(now){
    const dt=previousNow?Math.min(.25,(now-previousNow)/1000):0;previousNow=now;
    if(demo.playing&&!document.hidden){demo.time=Math.min(scenario.duration,demo.time+dt*demo.rate);if(demo.time>=scenario.duration)demo.playing=false;paint(false);}
    requestAnimationFrame(tick);
  }
  demo.attach=function(value){bridge=value;};
  demo.seek=seek;
  demo.play=function(){if(demo.time>=scenario.duration)seek(0);demo.playing=true;paint(true);};
  demo.pause=function(){demo.playing=false;paint(true);};
  demo.initialize=function(){
    if(initialized)return;initialized=true;document.body.classList.add('demo-mode','demo-recording');
    document.title='巡检小车 · 模拟演示 V8.14';
    const section=document.createElement('section');section.id='rover-demo-controls';
    section.innerHTML=`<style>
      #rover-demo-controls{margin:16px 0 18px;padding:13px 16px;background:#17251e;border:1px solid #41624f;border-radius:7px;color:#edf4ef}
      #rover-demo-controls .demo-line{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
      #rover-demo-controls .demo-label{color:#f0c77f;font-weight:600;font-size:13px;white-space:nowrap}
      #rover-demo-controls #demo-phase{color:#a4c6b1;font-size:12px;margin-right:auto}
      #rover-demo-controls button,#rover-demo-controls select{font:inherit;font-size:12px;background:#223b2c;border:1px solid #496652;border-radius:5px;color:#edf4ef;padding:7px 11px;cursor:pointer}
      #rover-demo-controls #demo-play{background:#4bda9a;color:#082016;font-weight:600;border-color:#4bda9a}
      #rover-demo-controls .timeline{display:flex;align-items:center;gap:12px;margin-top:10px}
      #rover-demo-controls input[type=range]{min-width:0;flex:1;accent-color:#4bda9a;cursor:pointer}
      #demo-clock{color:#a4bcad;font-size:12px;font-variant-numeric:tabular-nums;white-space:nowrap}
      #rover-demo-controls .stations{display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:8px;margin-top:10px}
      #rover-demo-controls .station{text-align:left;display:flex;align-items:center;gap:8px;padding:7px 10px;background:#142119;border-color:#304b39}
      #rover-demo-controls .station b{font-weight:500;color:#90ae9a;font-size:16px}#rover-demo-controls .station span{font-size:12px;line-height:1.3}#rover-demo-controls .station small{display:block;color:#92a69a;font-size:10px;margin-top:2px}
      #rover-demo-controls .station.complete b{color:#ff7777}#rover-demo-controls .station.current{border-color:#4bda9a;background:#1b3929}
      .demo-mode .top-actions{flex-wrap:wrap}.demo-mode #connect{display:none}.demo-mode #pause{display:none}
      .demo-mode .demo-watermark{position:fixed;bottom:12px;right:16px;color:#f0c77f;background:#142119ed;border:1px solid #6b6343;padding:5px 11px;border-radius:4px;font-size:12px;z-index:30;pointer-events:none}
      @media(min-width:1081px){.demo-recording .app{max-width:1740px;padding:12px 22px}.demo-recording .top{padding-bottom:10px}.demo-recording .layout{grid-template-columns:200px minmax(0,1fr) 220px;margin-top:12px;gap:14px}.demo-recording .scene-panel{height:310px;min-height:310px;padding:12px}.demo-recording .side{padding:11px}.demo-recording .side .divider{margin:10px 0}.demo-recording .right>.eyebrow:first-child,.demo-recording #targets,.demo-recording .right>.divider,.demo-recording .right .raw{display:none}.demo-recording .gas-list{margin-top:5px;gap:4px}.demo-recording .gas-card{padding:5px 8px}.demo-recording .gas-card strong{font-size:12px}.demo-recording .gas-card small{margin-top:1px;font-size:10px}.demo-recording .cache-head{margin-top:10px}.demo-recording .legend{font-size:10px;gap:10px}.demo-recording .bottom,.demo-recording #wheel-speed-panel{display:none}.demo-recording .distance{font-size:41px}.demo-recording .metric-list{gap:9px}.demo-recording #rover-demo-controls{margin:10px 0;padding:10px 14px}.demo-recording #vehicle-route-panel{margin-top:14px!important}.demo-recording .footer{margin-top:8px}}
      @media(max-width:680px){#rover-demo-controls{padding:12px}.demo-mode .top-actions{display:flex}#rover-demo-controls .stations{grid-template-columns:repeat(2,minmax(0,1fr))}#rover-demo-controls .demo-line{gap:7px}#demo-phase{flex-basis:100%;order:2}}
    </style><div class="demo-line"><strong class="demo-label">模拟数据 · 离线演示</strong><span id="demo-phase">准备出发</span><button id="demo-play">开始演示</button><button id="demo-restart">从头开始</button><label style="font-size:12px;color:#a4bcad">速度 <select id="demo-rate" aria-label="演示播放速度"><option value="0.5">0.5×</option><option value="1" selected>1×</option><option value="2">2×</option><option value="4">4×</option></select></label><button id="demo-layout" aria-pressed="true">录屏布局：开</button><button id="demo-export">下载完整模拟路线</button></div><div class="timeline"><input id="demo-progress" type="range" min="0" max="${scenario.duration}" step="0.08" value="0" aria-label="模拟演示进度"><span id="demo-clock"></span></div><div class="stations">${STATIONS.map((s,i)=>`<button class="station" data-station="${i}"><b>${i+1}</b><span>${s.name}<small>待扫描</small></span></button>`).join('')}</div>`;
    document.querySelector('.top').after(section);ui=section;
    const watermark=document.createElement('div');watermark.className='demo-watermark';watermark.textContent='模拟演示 · 非实测数据';document.body.append(watermark);
    section.querySelector('#demo-play').onclick=()=>demo.playing?demo.pause():demo.play();
    section.querySelector('#demo-restart').onclick=()=>{seek(0);demo.playing=true;paint(true);};
    section.querySelector('#demo-rate').onchange=e=>demo.rate=Number(e.target.value);
    section.querySelector('#demo-progress').oninput=e=>seek(Number(e.target.value));
    section.querySelector('#demo-layout').onclick=e=>{const active=document.body.classList.toggle('demo-recording');e.target.textContent='录屏布局：'+(active?'开':'关');e.target.setAttribute('aria-pressed',String(active));root.dispatchEvent(new Event('resize'));};
    section.querySelectorAll('[data-station]').forEach((button,i)=>button.onclick=()=>{
      demo.pause();seek(scenario.checkpoints[i].at+.08);root.VehicleRoute.openDemoScan(i+1);
    });
    section.querySelector('#demo-export').onclick=()=>{
      const route=scenario.routeAt(scenario.duration,origin),record=root.VehicleRouteCore.routeRecord(route);
      const url=URL.createObjectURL(new Blob([JSON.stringify(record,null,2)],{type:'application/json'}));
      const a=document.createElement('a');a.href=url;a.download='模拟数据_完整巡检路线_五个扫描点.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
    };
    const footer=document.querySelector('.footer>span');footer.textContent='V8.14 · 模拟路线、测距、气体与轮速，仅用于功能演示';
    const sceneLabel=document.querySelector('.scene-head strong');sceneLabel.textContent='三维扫描空间 · 模拟数据';
    root.addEventListener('gimbal:snapshot-open',()=>{if(demo.playing)demo.pause();});
    if(bridge.view)bridge.view();
    root.dispatchEvent(new Event('resize'));
    paint(true);requestAnimationFrame(tick);
  };
})(typeof globalThis!=='undefined'?globalThis:this);
