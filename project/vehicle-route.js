/* V8.11 / 2026-09-21. Route markers keep independent ultrasonic scan snapshots.
 * Route JSON v2 stores scans; v1 routes remain readable. Browser-only, wiring unchanged.
 * 80ms count windows + X-mecanum consistency checks; no motor commands.
 * E1 counters are cumulative uint32, A-rising (1x); never multiply CPR by 4.
 * X mecanum: body x=forward, y=left, heading positive=counterclockwise.
 * Legacy E1 odometry remains for older firmware. Neither source is SLAM.
 */
(function (root) {
  'use strict';
  const u32 = n => Number.isInteger(n) && n >= 0 && n <= 4294967295;
  const wrapDelta = (a, b) => (a - b) | 0;
  const wrapAngle = a => Math.atan2(Math.sin(a), Math.cos(a));
  // User: diameter 97 mm, track 202 mm, wheelbase 210 mm; 34:1 x 11 PPR = 374 (1x).
  // Direction/layout confirmation remains manual; supplied ratio is not physical calibration.
  const defaults = () => ({ diameter: 97, cpr: 374, track: 202, wheelbase: 210, signs: [1,-1,1,-1], confirmed: false });
  function validConfig(c) {
    return c && Number.isFinite(c.diameter) && c.diameter >= 20 && c.diameter <= 500 &&
      Number.isFinite(c.cpr) && c.cpr >= 1 && c.cpr <= 1000000 &&
      Number.isFinite(c.track) && c.track >= 30 && c.track <= 2000 &&
      Number.isFinite(c.wheelbase) && c.wheelbase >= 30 && c.wheelbase <= 2000 &&
      Array.isArray(c.signs) && c.signs.length === 4 && c.signs.every(v => v === 1 || v === -1) &&
      c.confirmed === true;
  }
  function validSample(s) { return Array.isArray(s) && s.length === 6 && s.every(u32); }
  class RouteEngine {
    constructor(config) {
      this.config = config || defaults();
      this.session = null; this.last = null; this.raw = [0,0,0,0];
      this.online = false; this.lastReceived = 0; this.errors = 0;
      this.reset();
    }
    reset() {
      this.x=0; this.y=0; this.heading=0; this.distance=0; this.speed=0;
      this.rpm=[0,0,0,0]; this.points=[]; this.scans=[]; this.elapsed=0; this.gaps=0;
      this.restarts=0; this.decimations=0; this.travel=[0,0,0,0];
      this.last=null; this.originTime=Math.max(Date.now(),(this.originTime||0)+1); this.breakNext=true;
      this.pendingWheel=[0,0,0,0];this.pendingMs=0;this.pendingBad=false;
    }
    configure(config) {
      if (!validConfig(config)) throw new Error('请填写真实尺寸、每轮一圈计数，并确认四轮方向和X型安装。');
      this.config=JSON.parse(JSON.stringify(config)); this.reset();
    }
    addPoint(force, gap) {
      const p=this.points[this.points.length-1];
      if(!force && p && Math.hypot(this.x-p.x,this.y-p.y)<0.015 &&
         Math.abs(wrapAngle(this.heading-p.heading))<0.035)return;
      this.points.push({x:this.x,y:this.y,heading:this.heading,time:this.elapsed,
        distance:this.distance,gap:!!gap});
      if(this.points.length>10000) {
        // Keep endpoints and every gap boundary, simplify only drawing history.
        this.points=this.points.filter((p,i,a)=>i===0||i===a.length-1||i%2===0||p.gap||(a[i+1]&&a[i+1].gap));
        if(this.points.length>10000)this.points=this.points.slice(-10000);
        this.decimations++;
      }
    }
    feed(odo, now=Date.now()) {
      if(!odo || odo.protocol!==1 || typeof odo.session!=='string' ||
         !Array.isArray(odo.samples) || odo.samples.length>16 || !odo.samples.every(validSample)) {
        this.online=false;this.speed=0;this.rpm=[0,0,0,0];return;
      }
      if(odo.online!==true){this.online=false;this.speed=0;this.rpm=[0,0,0,0];return;}
      if(this.session!==odo.session) {
        if(this.session!==null) {this.restarts++;this.breakNext=true;}
        this.session=odo.session;this.last=null;
        this.pendingWheel=[0,0,0,0];this.pendingMs=0;this.pendingBad=false;
      }
      this.online=true;this.errors=Number(odo.errors)||0;
      let samples=odo.samples;
      // On first connection/reset start HERE, never replay old buffered travel.
      if(!this.last && samples.length)samples=[samples[samples.length-1]];
      for(const sample of samples) {
        if(this.last) {
          const ds=(sample[0]-this.last[0])>>>0;
          if(ds===0||ds>=2147483648)continue;
        }
        this.raw=sample.slice(2).map(v=>v|0);
        this.lastReceived=now;
        if(!this.last) {
          this.last=sample.slice(); this.speed=0;this.rpm=[0,0,0,0];
          if(validConfig(this.config))this.addPoint(true,true);
          this.breakNext=false;
          continue;
        }
        const previous=this.last,dt=(sample[1]-previous[1])>>>0;
        this.last=sample.slice();
        if(!dt || dt>=2147483648) {
          this.pendingWheel=[0,0,0,0];this.pendingMs=0;this.pendingBad=false;
          this.gaps++;this.breakNext=true;this.speed=0;this.rpm=[0,0,0,0];continue;
        }
        const ticks=sample.slice(2).map((v,i)=>wrapDelta(v,previous[i+2]));
        if(!validConfig(this.config)) {this.speed=0;continue;}
        const c=this.config;
        const tickM=Math.PI*c.diameter/(1000*c.cpr);
        const sampleWheel=ticks.map((n,i)=>n*c.signs[i]*tickM);
        this.elapsed+=dt;
        this.rpm=ticks.map((n,i)=>n*c.signs[i]*60000/(c.cpr*dt));
        sampleWheel.forEach((d,i)=>{this.travel[i]+=d;this.pendingWheel[i]+=d;});
        this.pendingMs+=dt;
        // Local E1 fallback cannot verify an unknown bend after a long outage.
        this.pendingBad ||= dt>320||sampleWheel.some(v=>Math.abs(v)>2*dt/1000);
        if(this.pendingMs<80)continue;
        const wheel=this.pendingWheel,windowMs=this.pendingMs;
        const residual=Math.abs(wheel[0]+wheel[1]-wheel[2]-wheel[3]);
        const bad=this.pendingBad||residual>4*tickM+.15*wheel.reduce((s,v)=>s+Math.abs(v),0);
        this.pendingWheel=[0,0,0,0];this.pendingMs=0;this.pendingBad=false;
        if(bad){this.gaps++;this.speed=0;this.addPoint(true,true);this.breakNext=true;continue;}
        const gap=false;
        const dx=(wheel[0]+wheel[1]+wheel[2]+wheel[3])/4;
        const dy=(-wheel[0]+wheel[1]+wheel[2]-wheel[3])/4;
        const k=(c.track+c.wheelbase)/2000;
        const da=(-wheel[0]+wheel[1]-wheel[2]+wheel[3])/(4*k);
        // SE(2) exponential for a constant body twist during this sample.
        const a=Math.abs(da)<1e-8?1:Math.sin(da)/da;
        const q=Math.abs(da)<1e-8?0:(1-Math.cos(da))/da;
        const bx=a*dx-q*dy,by=q*dx+a*dy,cos=Math.cos(this.heading),sin=Math.sin(this.heading);
        this.x+=cos*bx-sin*by;this.y+=sin*bx+cos*by;
        this.heading=wrapAngle(this.heading+da);
        const length=Math.hypot(dx,dy);
        this.distance+=length; // Pure turn is zero body-centre travel.
        this.speed=length/(windowMs/1000);
        this.addPoint(gap||this.breakNext,gap||this.breakNext);
        this.breakNext=false;
      }
      if(now-this.lastReceived>1200){this.online=false;this.speed=0;this.rpm=[0,0,0,0];}
    }
    stale(now=Date.now()) {
      if(now-this.lastReceived>1200){this.online=false;this.speed=0;this.rpm=[0,0,0,0];}
    }
  }
  const MAX_ROUTE_BYTES=20*1024*1024;
  // ponytail: bound in-memory inspection files; split long surveys into route files.
  const MAX_SCANS=100,MAX_SCAN_POINTS=800;
  const finite=(v,low,high)=>typeof v==='number'&&Number.isFinite(v)&&v>=low&&v<=high;
  function cleanScan(scan){
    const check=ok=>{if(!ok)throw Error('扫描点图格式不正确或数据超出范围。');};
    check(scan&&(scan.kind==='2d'||scan.kind==='3d')&&Array.isArray(scan.points)&&
      scan.points.length>0&&scan.points.length<=MAX_SCAN_POINTS);
    const points=scan.points.map(p=>{
      check(p&&finite(p.distance,2,150)&&finite(p.yaw,-180,180)&&finite(p.pitch,-180,180));
      return {distance:p.distance,yaw:p.yaw,pitch:p.pitch};
    });
    const copy={kind:scan.kind,points};
    if(scan.view!==undefined){
      check(scan.view&&['camera','target'].every(k=>Array.isArray(scan.view[k])&&
        scan.view[k].length===3&&scan.view[k].every(v=>finite(v,-10000,10000))));
      copy.view={camera:scan.view.camera.slice(),target:scan.view.target.slice()};
    }
    return copy;
  }
  function cleanScans(scans,route){
    if(scans===undefined)return [];
    if(!Array.isArray(scans)||scans.length>MAX_SCANS)throw Error('每条路线最多保存100个扫描点。');
    const ids=new Set();
    return scans.map(s=>{
      const p=s&&s.pose;
      if(!s||!u32(s.id)||!s.id||ids.has(s.id)||!finite(s.capturedAt,0,8640000000000000)||
         !p||!finite(p.x,-1e6,1e6)||!finite(p.y,-1e6,1e6)||!finite(p.heading,-Math.PI,Math.PI)||
         !finite(p.time,0,route.elapsed)||!finite(p.distance,0,route.distance))
        throw Error('扫描点的位置、时间或编号不正确。');
      ids.add(s.id);
      return {id:s.id,capturedAt:s.capturedAt,
        pose:{x:p.x,y:p.y,heading:p.heading,time:p.time,distance:p.distance},scan:cleanScan(s.scan)};
    });
  }
  function captureScan(route,scan,now=Date.now()){
    if(!route||!validConfig(route.config)||!route.points.length)
      throw Error('请先连接基座并确认路线标定，获得当前位置后再保存扫描点。');
    if(route.online!==true||!finite(route.lastReceived,now-1200,now)||
       (route.nativeLog&&route.logPending))
      throw Error('当前位置尚未同步，请停车并等待路线数据同步后再保存扫描点。');
    const checked=cleanRoute(route);
    if(checked.scans.length>=MAX_SCANS)throw Error('已达到100个扫描点，请先保存路线，再开始新路线。');
    const id=checked.scans.reduce((max,s)=>Math.max(max,s.id),0)+1;
    const checkpoint={id,capturedAt:now,
      pose:{x:route.x,y:route.y,heading:route.heading,time:route.elapsed,distance:route.distance},
      scan:cleanScan(scan)};
    // Validate before changing the live route; imported data and live arrays never alias.
    const scans=cleanScans([...checked.scans,checkpoint],checked);
    route.scans=scans;
    return scans[scans.length-1];
  }
  function cleanRoute(r){
    const num=(v,low=-1e6,high=1e6)=>typeof v==='number'&&Number.isFinite(v)&&v>=low&&v<=high;
    const check=(ok)=>{if(!ok)throw new Error('路线文件格式不正确或数据超出范围。');};
    check(r&&validConfig(r.config)&&num(r.x)&&num(r.y)&&num(r.heading,-Math.PI,Math.PI)&&
      num(r.distance,0,1e9)&&num(r.elapsed,0,1e12)&&num(r.originTime,0,8640000000000000));
    check(r.session===null||(typeof r.session==='string'&&r.session.length<=128));
    ['gaps','restarts','decimations'].forEach(k=>check(Number.isInteger(r[k])&&num(r[k],0,1e9)));
    check(Array.isArray(r.points)&&r.points.length>0&&r.points.length<=10000);
    let lastTime=0,lastDistance=0;
    const points=r.points.map(p=>{
      check(p&&num(p.x)&&num(p.y)&&num(p.heading,-Math.PI,Math.PI)&&num(p.time,lastTime,r.elapsed)&&
        num(p.distance,lastDistance,r.distance)&&typeof p.gap==='boolean');
      lastTime=p.time;lastDistance=p.distance;
      return {x:p.x,y:p.y,heading:p.heading,time:p.time,distance:p.distance,gap:p.gap};
    });
    const c=r.config;
    return {x:r.x,y:r.y,heading:r.heading,distance:r.distance,elapsed:r.elapsed,originTime:r.originTime,
      session:r.session,gaps:r.gaps,restarts:r.restarts,decimations:r.decimations,points,scans:cleanScans(r.scans,r),
      config:{diameter:c.diameter,cpr:c.cpr,track:c.track,wheelbase:c.wheelbase,confirmed:true,signs:c.signs.slice()}};
  }
  function routeRecord(route,now=Date.now()){
    return {format:'gimbal-route',version:2,savedAt:now,route:cleanRoute(route)};
  }
  function parseRouteFile(raw){
    if(typeof raw!=='string'||new TextEncoder().encode(raw).length>MAX_ROUTE_BYTES)throw new Error('路线文件不能超过20MB。');
    let data;try{data=JSON.parse(raw.replace(/^\uFEFF/,''));}catch(_){throw new Error('请选择保存的路线 JSON 文件。');}
    if(!data||!((data.format==='gimbal-route'&&(data.version===1||data.version===2))||
       (data.format==='gimbal-inspection'&&data.version===1)))
      throw new Error('不支持这个文件类型或版本。');
    // Legacy inspection files retain their route; unrelated old station records are ignored.
    return cleanRoute(data.route);
  }
  const exported={RouteEngine,validConfig,defaults,wrapDelta,cleanRoute,cleanScan,captureScan,
    routeRecord,parseRouteFile,MAX_ROUTE_BYTES,MAX_SCANS,MAX_SCAN_POINTS};
  if(typeof module==='object'&&module.exports)module.exports=exported;
  root.VehicleRouteCore=exported;
  if(typeof document==='undefined')return;
  let config=defaults();
  try { const saved=JSON.parse(localStorage.getItem('gimbal-route-calibration-v1'));if(validConfig(saved)){
    if(saved.signs.join(',')==='-1,1,-1,1')saved.signs=[1,-1,1,-1];
    config=saved;
  } } catch (_) {}
  const engine=new RouteEngine(config);
  const rawEngine=new RouteEngine(config);
  const host=document.createElement('section');
  host.id='vehicle-route-panel';
  host.style.cssText='display:block;grid-column:1/-1;margin:22px 0 0;';
  const shadow=host.attachShadow({mode:'open'});
  shadow.innerHTML=`
    <style>
      :host{font:14px/1.5 "Segoe UI","Microsoft YaHei",sans-serif;color:#eaf4ef}
      *{box-sizing:border-box}button,input,select{font:inherit}
      .card{border:1px solid #354b40;border-radius:12px;background:#16211c;overflow:hidden}
      header{padding:21px 24px;display:flex;gap:16px;align-items:center;justify-content:space-between;border-bottom:1px solid #2c4035}
      h2{font-size:19px;letter-spacing:1px;margin:0 0 3px}p{margin:0;color:#92ab9c;font-size:12px}
      .tag{padding:6px 11px;border-radius:20px;border:1px solid #886c3d;background:#30291d;color:#f0c77f;font-size:12px;white-space:nowrap}
      .tag.live{color:#7ce4b0;background:#1a3528;border-color:#35634b}
      .body{display:grid;grid-template-columns:minmax(0,1fr) 278px}
      .map{position:relative;min-width:0;background:radial-gradient(ellipse at 45% 30%,#1a2e23,#0e1812)}
      canvas{display:block;width:100%;height:380px;touch-action:none;cursor:grab}
      .map-hint{padding:8px 16px;color:#88a294;font-size:11px;pointer-events:none}
      .toolbar{display:flex;gap:7px;flex-wrap:wrap;padding:12px 16px;border-top:1px solid #2a3c31}
      .files{display:flex;align-items:center;gap:8px;flex-wrap:wrap;padding:12px 16px;border-bottom:1px solid #2a3c31}
      .snapshots{display:flex;align-items:center;gap:8px;flex-wrap:wrap;padding:10px 16px;border-bottom:1px solid #2a3c31}
      .snapshots label{display:flex;flex-direction:row;align-items:center;gap:7px}.snapshots select{width:auto;max-width:250px;padding:6px}
      .snapshots small{flex:1 1 230px}.scan-dot{color:#ff7676}
      .file-status{flex:1 1 180px;min-width:0;overflow-wrap:anywhere;color:#a3b7aa;font-size:12px}
      [hidden]{display:none!important}button:disabled{opacity:.45;cursor:not-allowed}
      button{border:1px solid #3b5143;color:#dcebe2;background:#213228;border-radius:6px;padding:7px 12px;cursor:pointer}
      button:hover{background:#2f4838}button:focus-visible,input:focus-visible,select:focus-visible{outline:2px solid #65dfa4;outline-offset:2px}
      .primary{background:#57d39a;color:#10251a;border-color:#57d39a;font-weight:600}
      .metrics{padding:20px;border-left:1px solid #2d4035;display:flex;flex-direction:column;gap:17px}
      .total{font-size:35px;line-height:1.15;letter-spacing:-1px;font-variant-numeric:tabular-nums}
      small{font-size:12px;color:#8fa697}.row{display:flex;justify-content:space-between;gap:8px}
      strong{font-variant-numeric:tabular-nums}.divider{height:1px;background:#2c4035}
      .wheels{display:grid;grid-template-columns:1fr 1fr;gap:8px}
      .wheel{background:#1d2b23;border:1px solid #304437;border-radius:6px;padding:8px;min-width:0}
      .wheel b,.wheel span{display:block;overflow-wrap:anywhere}.wheel span{font-size:11px;color:#8fa697}
      .note{padding:12px 20px;background:#122019;border-top:1px solid #2c4035;color:#a3b7aa;font-size:12px;min-height:43px}
      details{border-top:1px solid #2c4035;padding:16px 22px;background:#18251d}summary{cursor:pointer;font-weight:600}
      .form{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px;margin-top:16px}
      label{display:flex;flex-direction:column;gap:5px;font-size:12px;color:#a8bdb0}
      input,select{width:100%;min-width:0;color:#eef8f2;background:#101c14;border:1px solid #3b5244;padding:9px;border-radius:5px}
      input[type=checkbox]{width:auto}.check{display:flex;flex-direction:row;align-items:flex-start;gap:8px;margin:14px 0}
      .help{font-size:12px;color:#9bb4a3;margin:12px 0}
      .error{color:#ffc080;min-height:20px;margin-top:10px;font-size:12px}.smallbtn{padding:4px 9px;font-size:12px}
      @media(max-width:760px){header{align-items:flex-start;padding:16px}.body{grid-template-columns:1fr}.metrics{border-left:0;border-top:1px solid #2d4035;gap:12px}.form{grid-template-columns:1fr 1fr}canvas{height:330px}.total{font-size:29px}}
    </style>
    <div class="card">
      <header><div><h2>巡检行驶轨迹</h2><p>四轮编码器 · 相对起点的平面路线</p></div><span id="state" class="tag">等待编码器数据</span></header>
      <div class="files"><button id="saveRoute" class="primary">保存路线</button><button id="openRoute">打开路线</button><button id="liveRoute" hidden>返回实时</button><span id="routeFileStatus" class="file-status" role="status">当前路线</span><input id="routeFile" type="file" accept=".json,application/json" hidden></div>
      <div class="snapshots"><button id="saveScan">保存当前扫描点</button><label><span class="scan-dot">●</span> 扫描点<select id="scanSelect" aria-label="已保存扫描点"><option value="">暂无扫描点</option></select></label><button id="viewScan" disabled>查看点图</button><small id="scanStatus" role="status">停车完成所需扫描后保存，点击路线红点回看。</small></div>
      <div class="body">
        <div class="map"><canvas id="map" aria-label="小车编码器估算行驶轨迹图"></canvas><div class="map-hint" id="hint">起点朝上 · 左为车体左侧 · 拖动平移 / 滚轮缩放</div>
          <div class="toolbar"><button id="fit">查看全程</button><button id="follow" aria-pressed="true">跟随小车：开</button><button id="zoomIn" aria-label="放大轨迹">＋</button><button id="zoomOut" aria-label="缩小轨迹">－</button><button id="reset">以当前位置重新开始</button><button id="export">导出轨迹</button></div>
        </div>
        <aside class="metrics">
          <div><small>本次累计路程</small><div class="total"><span id="distance">--</span> <small>m</small></div></div>
          <div class="row"><small>距起点</small><strong id="away">-- m</strong></div>
          <div class="row"><small>车体速度</small><strong id="speed">-- m/s</strong></div>
          <div class="row"><small>相对朝向（左转为正）</small><strong id="heading">--°</strong></div>
          <div class="divider"></div><div><small>四轮原始计数 / 转速</small><div class="wheels" id="wheels"></div></div>
          <p>轮子不转时计数不变；零计数不能证明编码器已接好。</p>
        </aside>
      </div>
      <div id="note" class="note" role="status">先连接基座，再展开下方标定设置。尚未填写参数时只显示计数。</div>
      <details id="calibration"><summary>轨迹标定与轮子方向</summary>
        <div class="form">
          <label>轮子直径（mm）<input id="diameter" type="number" min="20" max="500" step="any" placeholder="填写实测轮径"></label>
          <label>轮子一圈计数（1倍频）<input id="cpr" type="number" min="1" max="1000000" step="any" placeholder="通常为 11 × 减速比"></label>
          <label>左右轮中心间距（mm）<input id="track" type="number" min="30" max="2000" step="any" placeholder="轮距"></label>
          <label>前后轮中心间距（mm）<input id="wheelbase" type="number" min="30" max="2000" step="any" placeholder="轴距"></label>
        </div>
        <p class="help">先转动车轮检查上方原始计数，再设置方向：每个轮子朝“让车前进”的方向转动时，原始计数增加选 +1，减少选 −1。更改参数会从当前位置重新计程。</p>
        <div class="form" id="signs"></div>
        <label class="check"><input type="checkbox" id="confirmed">我已核对四轮计数方向及标准 X 型麦轮安装（俯视滚子轴线：前排 ＼ ／，后排 ／ ＼）。</label>
        <p class="help">本固件只数 A 相上升沿，不是四倍频。可手动转轮 10 圈，用计数差的绝对值 ÷ 10 填“轮子一圈计数”。四个轮子应同规格。参数保存在当前浏览器，不发送电机指令。</p>
        <button id="apply" class="primary">保存标定并重新开始</button>
        <div id="formError" class="error" role="status"></div>
      </details>
    </div>`;
  const $=id=>shadow.getElementById(id);
  const names=['1 左前','2 右前','3 左后','4 右后'];
  $('wheels').innerHTML=names.map((n,i)=>'<div class="wheel"><span>'+n+'</span><b id="count'+i+'">--</b><span id="rpm'+i+'">-- 转/分</span></div>').join('');
  $('signs').innerHTML=names.map((n,i)=>'<label>'+n+'方向<select id="sign'+i+'"><option value="1">+1 原始计数增加</option><option value="-1">−1 原始计数减少</option></select></label>').join('');
  ['diameter','cpr','track','wheelbase'].forEach(k=>{$(k).value=config[k]||'';});
  config.signs.forEach((n,i)=>{$('sign'+i).value=n;});$('confirmed').checked=config.confirmed;
  const main=document.querySelector('main');
  if(main){const footer=main.querySelector('.footer');if(footer)footer.before(host);else main.appendChild(host);}
  else document.body.appendChild(host);
  const canvas=$('map'),ctx=canvas.getContext('2d');
  let scale=85,center={x:0,y:0},follow=true,drag=null,dirty=true,lastPaint=0;
  let historyRoute=null,saveDirectory=null,saving=false,opening=false;
  let scanCapture=null,selectedScan=0,scanListRoute=null,scanListLength=-1;
  const shownRoute=()=>historyRoute||engine;
  const fixed=(v,n=2)=>Number.isFinite(v)?v.toFixed(n):'--';
  function refresh() {
    const shown=shownRoute();
    const calibrated=validConfig(shown.config),live=shown.online;
    $('state').textContent=!live?'编码器数据离线':!calibrated?'数据在线 · 待确认轮向':'编码器估算中';
    if(historyRoute)$('state').textContent='已保存路线';
    $('reset').disabled=$('apply').disabled=!!historyRoute;
    $('liveRoute').hidden=!historyRoute;
    $('saveScan').disabled=!!historyRoute||!scanCapture;
    refreshScans(shown);
    $('state').className='tag'+(live&&calibrated?' live':'');
    $('distance').textContent=calibrated?fixed(shown.distance):'--';
    $('away').textContent=(calibrated?fixed(Math.hypot(shown.x,shown.y)):'--')+' m';
    $('speed').textContent=(calibrated&&live?fixed(shown.speed):'--')+' m/s';
    $('heading').textContent=(calibrated?fixed(shown.heading*180/Math.PI,1):'--')+'°';
    const rawReady=engine.nativeLog&&!historyRoute?!!rawEngine.last:!!shown.session;
    names.forEach((_,i)=>{$('count'+i).textContent=rawReady&&shown.raw?String(shown.raw[i]):'--';$('rpm'+i).textContent=(calibrated&&live&&shown.rpm?fixed(shown.rpm[i],1):'--')+' 转/分';});
    let note=!live?'等待新的编码器数据，旧路线保留。检查主控 V6.5 与基座 V4.4 是否已烧录。':
      !calibrated?'已收到四轮计数，已预填轮径97mm、轮距202mm、轴距210mm及374次/轮圈。请核对四轮方向后保存标定。':
      '编码器估算轨迹：以开始记录时车头方向为前方。麦轮打滑会积累误差，不能作为绝对定位。';
    if(shown.gaps)note+=' 存在 '+shown.gaps+' 处数据缺口/异常：路线已分段，路程可能低估。';
    if(shown.restarts)note+=' 记录会话发生切换，可能是设备重启或数据源变化；切换前后路线不连接。';
    if(shown.decimations)note+=' 长路线已简化绘制点，累计路程保持。';
    if(engine.nativeLog&&!historyRoute){
      $('state').textContent=engine.logPending?'路线同步中':live?'车端记录已同步':'路线连接中断';
      note=engine.logMessage+'。编码器估算，打滑仍会积累误差。';
      if(shown.gaps||shown.restarts)note+=' 存在 '+shown.gaps+' 处采样/缓存缺口、'+shown.restarts+' 次车端会话切换；缺失位置不作已知路线。';
    }
    if(historyRoute)note='已保存路线 · 编码器估算，不是绝对定位。';
    $('note').textContent=note;
    dirty=true;
  }
  function refreshScans(route){
    const scans=route.scans||[];
    if(scanListRoute!==route||scanListLength!==scans.length){
      if(scanListRoute!==route)selectedScan=0;
      scanListRoute=route;scanListLength=scans.length;
      $('scanSelect').replaceChildren();
      const empty=document.createElement('option');empty.value='';empty.textContent=scans.length?'选择扫描点':'暂无扫描点';$('scanSelect').append(empty);
      scans.forEach(s=>{const option=document.createElement('option');option.value=String(s.id);
        option.textContent='#'+s.id+' · '+new Date(s.capturedAt).toLocaleTimeString('zh-CN',{hour12:false})+' · '+s.scan.points.length+'点';$('scanSelect').append(option);});
    }
    if(!scans.some(s=>s.id===selectedScan))selectedScan=0;
    $('scanSelect').value=selectedScan?String(selectedScan):'';
    $('viewScan').disabled=!selectedScan;
  }
  function showScan(id){
    const s=(shownRoute().scans||[]).find(s=>s.id===id);
    if(!s)return;
    selectedScan=id;refreshScans(shownRoute());dirty=true;
    try{
      if(!root.ScanSnapshotViewer)throw Error('点图查看器未加载，请保留 scan-snapshot.js 并刷新页面。');
      root.ScanSnapshotViewer.open(s);
    }catch(e){$('scanStatus').textContent=e.message;}
  }
  $('saveScan').onclick=()=>{
    if(historyRoute||!scanCapture)return;
    try{
      const s=captureScan(engine,scanCapture());selectedScan=s.id;refresh();
      $('scanStatus').textContent='已记录扫描点 #'+s.id+'（'+s.scan.points.length+'点）。点击“保存路线”写入文件。';
    }catch(e){$('scanStatus').textContent=e.message;}
  };
  $('scanSelect').onchange=()=>{selectedScan=Number($('scanSelect').value);refreshScans(shownRoute());dirty=true;if(selectedScan)showScan(selectedScan);};
  $('viewScan').onclick=()=>showScan(selectedScan);
  function draw() {
    const shown=shownRoute();
    const rect=canvas.getBoundingClientRect(),w=rect.width,h=rect.height,dpr=Math.min(devicePixelRatio||1,2);
    if(w<1||h<1)return;
    if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
    ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
    if(follow)center={x:shown.x,y:shown.y};
    const screen=(x,y)=>({x:w/2-(y-center.y)*scale,y:h/2-(x-center.x)*scale});
    const candidates=[.05,.1,.2,.5,1,2,5,10,20,50,100,200,500,1000];
    const step=candidates.find(v=>v*scale>=44)||1000,grid=step*scale;
    const origin=screen(0,0);
    ctx.lineWidth=1;ctx.strokeStyle='#263d2e';ctx.beginPath();
    for(let x=((origin.x%grid)+grid)%grid;x<w;x+=grid){ctx.moveTo(x,0);ctx.lineTo(x,h);}
    for(let y=((origin.y%grid)+grid)%grid;y<h;y+=grid){ctx.moveTo(0,y);ctx.lineTo(w,y);}
    ctx.stroke();ctx.font='11px "Segoe UI",sans-serif';ctx.fillStyle='#799986';ctx.fillText('每格 '+step+' m',16,23);
    if(!validConfig(shown.config)){
      ctx.fillStyle='#c5dccd';ctx.font='16px "Microsoft YaHei",sans-serif';ctx.textAlign='center';
      ctx.fillText('已预填尺寸 · 确认四轮方向后开始绘图',w/2,h/2-6);
      ctx.font='12px "Microsoft YaHei",sans-serif';ctx.fillStyle='#819d8b';ctx.fillText('先检查四个轮子的累计计数',w/2,h/2+22);ctx.textAlign='left';return;
    }
    ctx.lineWidth=2.5;ctx.strokeStyle='#62d8a0';ctx.lineJoin='round';ctx.beginPath();
    shown.points.forEach((p,i)=>{const q=screen(p.x,p.y);if(i===0||p.gap)ctx.moveTo(q.x,q.y);else ctx.lineTo(q.x,q.y);});ctx.stroke();
    if(shown.points.length){const p=shown.points[0],q=screen(p.x,p.y);ctx.fillStyle='#8aa6ff';ctx.beginPath();ctx.arc(q.x,q.y,5,0,Math.PI*2);ctx.fill();ctx.fillText('起点',q.x+9,q.y+4);}
    const car=screen(shown.x,shown.y);
    ctx.save();ctx.translate(car.x,car.y);ctx.rotate(-shown.heading);
    ctx.shadowColor='#52d59b';ctx.shadowBlur=14;ctx.fillStyle=shown.online?'#7af0b5':'#8ca497';
    ctx.beginPath();ctx.moveTo(0,-13);ctx.lineTo(9,10);ctx.lineTo(0,5);ctx.lineTo(-9,10);ctx.closePath();ctx.fill();ctx.restore();
    (shown.scans||[]).forEach(s=>{const q=screen(s.pose.x,s.pose.y);ctx.beginPath();ctx.arc(q.x,q.y,s.id===selectedScan?7:5,0,Math.PI*2);
      ctx.fillStyle='#ff6868';ctx.fill();ctx.strokeStyle=s.id===selectedScan?'#fff1e9':'#632d2d';ctx.lineWidth=2;ctx.stroke();
      ctx.fillStyle='#ffc3b6';ctx.fillText('#'+s.id,q.x+10,q.y-8);});
    ctx.fillStyle='#829e8c';ctx.fillText('前方 ↑',w-67,23);
  }
  function fitRoute() {
    const shown=shownRoute();
    const pts=[...(shown.points.length?shown.points:[{x:0,y:0}]),...(shown.scans||[]).map(s=>s.pose)];
    let loX=shown.x,hiX=shown.x,loY=shown.y,hiY=shown.y;
    pts.forEach(p=>{loX=Math.min(loX,p.x);hiX=Math.max(hiX,p.x);loY=Math.min(loY,p.y);hiY=Math.max(hiY,p.y);});
    center={x:(loX+hiX)/2,y:(loY+hiY)/2};
    const r=canvas.getBoundingClientRect();
    scale=Math.max(.1,Math.min(300,(r.width-100)/Math.max(1,hiY-loY),(r.height-80)/Math.max(1,hiX-loX)));
    follow=false;$('follow').textContent='跟随小车：关';$('follow').setAttribute('aria-pressed','false');dirty=true;
  }
  $('apply').onclick=()=>{
    if(historyRoute)return;
    const c=defaults();['diameter','cpr','track','wheelbase'].forEach(k=>{c[k]=Number($(k).value);});
    c.signs=names.map((_,i)=>Number($('sign'+i).value));c.confirmed=$('confirmed').checked;
    try {if(!validConfig(c))throw new Error('请填写真实尺寸并确认四轮方向。');
      if(engine.scans.length&&!root.confirm('重新标定会清空当前路线和扫描点。请先保存路线。仍要重新开始吗？'))return;
      engine.configure(c);
      if(root.RouteLogClient)root.RouteLogClient.reset();
      let saved=true;try{localStorage.setItem('gimbal-route-calibration-v1',JSON.stringify(c));}catch(_){saved=false;}
      $('formError').textContent=saved?'已保存，从下一包数据开始记录。':'参数已应用；浏览器禁止本地存储，关闭后需要重填。';
      center={x:0,y:0};refresh();
    } catch(e){$('formError').textContent=e.message;}
  };
  function restart(){
    if(historyRoute)return;
    if(engine.scans.length&&!root.confirm('重新开始会清空当前路线和扫描点。请先保存路线。仍要继续吗？'))return;
    engine.reset();center={x:0,y:0};
    selectedScan=0;$('scanStatus').textContent='已重新开始，当前路线暂无扫描点。';
    if(root.RouteLogClient)root.RouteLogClient.reset();
    refresh();
  }
  $('reset').onclick=restart;
  $('fit').onclick=fitRoute;
  $('follow').onclick=()=>{follow=!follow;$('follow').textContent='跟随小车：'+(follow?'开':'关');$('follow').setAttribute('aria-pressed',String(follow));dirty=true;};
  $('zoomIn').onclick=()=>{scale=Math.min(1500,scale*1.35);dirty=true;};
  $('zoomOut').onclick=()=>{scale=Math.max(.1,scale/1.35);dirty=true;};
  canvas.onwheel=e=>{e.preventDefault();scale=Math.max(.1,Math.min(1500,scale*(e.deltaY<0?1.12:1/1.12)));dirty=true;};
  canvas.onpointerdown=e=>{drag={x:e.clientX,y:e.clientY,cx:center.x,cy:center.y,moved:false};canvas.setPointerCapture(e.pointerId);follow=false;$('follow').textContent='跟随小车：关';$('follow').setAttribute('aria-pressed','false');};
  canvas.onpointermove=e=>{if(!drag)return;drag.moved ||= Math.hypot(e.clientX-drag.x,e.clientY-drag.y)>5;if(!drag.moved)return;center={x:drag.cx+(e.clientY-drag.y)/scale,y:drag.cy+(e.clientX-drag.x)/scale};dirty=true;};
  canvas.onpointerup=e=>{
    const click=drag&&!drag.moved;drag=null;if(!click)return;
    const r=canvas.getBoundingClientRect(),px=e.clientX-r.left,py=e.clientY-r.top;
    const hit=[...(shownRoute().scans||[])].reverse().find(s=>Math.hypot(
      r.width/2-(s.pose.y-center.y)*scale-px,r.height/2-(s.pose.x-center.x)*scale-py)<=14);
    if(hit)showScan(hit.id);
  };
  canvas.onpointercancel=()=>{drag=null;};
  $('export').onclick=()=>{
    const shown=shownRoute();
    if(!validConfig(shown.config)||!shown.points.length){$('note').textContent='尚无已标定轨迹可导出。';return;}
    const lines=['# 编码器估算轨迹；x前方/y左方；缺口分段；不是绝对定位',
      '# 标定='+JSON.stringify(shown.config),
      '# 缺口='+shown.gaps+'; 重启='+shown.restarts+'; 绘制简化='+shown.decimations,
      'time_ms,x_m,y_m,heading_deg,distance_m,segment_start',
      ...shown.points.map(p=>[p.time,p.x.toFixed(5),p.y.toFixed(5),(p.heading*180/Math.PI).toFixed(3),p.distance.toFixed(5),p.gap?1:0].join(','))];
    const url=URL.createObjectURL(new Blob(['\uFEFF'+lines.join('\r\n')],{type:'text/csv;charset=utf-8'}));
    const a=document.createElement('a');a.href=url;a.download='巡检轨迹-'+new Date().toISOString().replace(/[:.]/g,'-')+'.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  };
  $('saveRoute').onclick=async()=>{
    if(saving)return;
    let text;
    try{
      if(!validConfig(shownRoute().config)||!shownRoute().points.length)throw new Error('暂无已标定路线可保存。');
      if(!historyRoute&&engine.nativeLog&&(engine.logPending||!engine.online||Date.now()-engine.lastReceived>1200))
        throw new Error('路线尚未同步完成，请先停车并等待“车端记录已同步”后保存。');
      text=JSON.stringify(routeRecord(shownRoute()),null,2);
      if(new TextEncoder().encode(text).length>MAX_ROUTE_BYTES)throw Error('路线与扫描点超过20MB，请缩短本次记录。');
    }catch(e){$('routeFileStatus').textContent=e.message;return;}
    const suffix=root.crypto&&root.crypto.randomUUID?root.crypto.randomUUID():Math.random().toString(36).slice(2);
    const filename='路线_'+new Date().toISOString().replace(/[:.]/g,'-')+'_'+suffix+'.json';
    saving=true;$('saveRoute').disabled=true;let writable=null;
    try{
      if(typeof root.showDirectoryPicker==='function'){
        if(!saveDirectory){
          $('routeFileStatus').textContent='请选择桌面的“路径保存数据库”文件夹';
          saveDirectory=await root.showDirectoryPicker({id:'gimbal-route',mode:'readwrite',startIn:'desktop'});
        }
        const permission={mode:'readwrite'};
        if(await saveDirectory.queryPermission(permission)!=='granted'&&await saveDirectory.requestPermission(permission)!=='granted')
          throw new Error('未获得文件夹写入权限，路线尚未保存。');
        try{await saveDirectory.getFileHandle(filename);throw new Error('同名文件已存在，请再次保存。');}
        catch(e){if(e.name!=='NotFoundError')throw e;}
        const handle=await saveDirectory.getFileHandle(filename,{create:true});
        writable=await handle.createWritable();await writable.write(text);await writable.close();writable=null;
        $('routeFileStatus').textContent='已保存到 '+saveDirectory.name+' / '+filename;
      }else if(typeof root.showSaveFilePicker==='function'){
        const handle=await root.showSaveFilePicker({suggestedName:filename,startIn:'desktop',types:[{description:'路线 JSON',accept:{'application/json':['.json']}}]});
        writable=await handle.createWritable();await writable.write(text);await writable.close();writable=null;
        $('routeFileStatus').textContent='已保存：'+handle.name;
      }else{
        const url=URL.createObjectURL(new Blob([text],{type:'application/json'})),a=document.createElement('a');
        a.href=url;a.download=filename;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
        $('routeFileStatus').textContent='已请求下载路线，请在下载记录中确认，并放入“路径保存数据库”。';
      }
    }catch(e){
      if(writable){try{await writable.abort();}catch(_){}}
      if(e.name==='NotAllowedError'||e.name==='SecurityError')saveDirectory=null;
      $('routeFileStatus').textContent=e.name==='AbortError'?'已取消，路线未保存。':'保存未完成：'+e.message;
    }finally{saving=false;$('saveRoute').disabled=false;}
  };
  $('openRoute').onclick=()=>{$('routeFile').value='';$('routeFile').click();};
  $('routeFile').onchange=async()=>{
    const file=$('routeFile').files[0];if(!file||opening)return;
    opening=true;$('openRoute').disabled=true;$('liveRoute').disabled=true;
    try{
      if(file.size>MAX_ROUTE_BYTES)throw new Error('路线文件不能超过20MB。');
      const route=parseRouteFile(await file.text());
      historyRoute=route;refresh();fitRoute();$('routeFileStatus').textContent=file.name;
      $('scanStatus').textContent='已打开 '+route.scans.length+' 个扫描点，点击红点或选择编号回看。';
    }catch(e){$('routeFileStatus').textContent='打开失败：'+e.message;}
    finally{opening=false;$('openRoute').disabled=false;$('liveRoute').disabled=false;}
  };
  $('liveRoute').onclick=()=>{
    historyRoute=null;follow=true;$('follow').textContent='跟随小车：开';$('follow').setAttribute('aria-pressed','true');
    $('routeFileStatus').textContent='当前路线';refresh();
    $('scanStatus').textContent='当前路线保留 '+engine.scans.length+' 个扫描点，点击“保存路线”写入文件。';
  };
  new ResizeObserver(()=>{dirty=true;}).observe(canvas);
  function paint(t){if(dirty&&t-lastPaint>50){draw();dirty=false;lastPaint=t;}requestAnimationFrame(paint);}
  requestAnimationFrame(paint);
  setInterval(()=>{engine.stale();refresh();},500);
  root.VehicleRoute={
    setScanCapture(fn){scanCapture=fn;refresh();},
    feed(odo){
      if(engine.nativeLog){
        rawEngine.config=engine.config;rawEngine.feed(odo);
        engine.raw=rawEngine.raw.slice();engine.rpm=rawEngine.rpm.slice();
      }else engine.feed(odo);
      refresh();
    },
    enableLog(){engine.nativeLog=true;engine.logPending=true;},
    legacyLog(){
      engine.nativeLog=false;engine.logPending=false;
      if(!engine.scans.length)engine.reset();
      else{
        // A disappearing log endpoint must not erase unsaved inspection snapshots.
        engine.last=null;engine.pendingWheel=[0,0,0,0];engine.pendingMs=0;engine.pendingBad=false;
        engine.breakNext=true;engine.online=false;engine.speed=0;engine.rpm=[0,0,0,0];engine.gaps++;
      }
      refresh();
    },
    refresh,
    disconnect(){engine.online=false;engine.speed=0;refresh();},
    selectedRoute:()=>historyRoute?cleanRoute(historyRoute):null,
    restart,engine
  };
  refresh();
})(typeof globalThis!=='undefined'?globalThis:this);
