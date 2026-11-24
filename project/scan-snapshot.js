/* V8.12 / 2026-09-21: saved scans use the same detailed gimbal model as the live page.
 * Webpage only; firmware, wiring, live scan arrays and control protocols unchanged.
 * Coordinates: distance cm, yaw=frame.roll, pitch=-frame.pitch, pose metres/radians.
 */
(function (root) {
  'use strict';
  let dismiss = null;
  const radians = degrees => degrees * Math.PI / 180;
  const color = distance => distance < 20 ? 0xff6d6d : distance < 40 ? 0xf0b95b : 0x4bda9a;
  const projection = (distance, pitch) => {
    const angle = radians(pitch);
    return {forward: distance * Math.cos(angle) - 6.5 * Math.sin(angle),
      height: distance * Math.sin(angle) + 6.5 * (Math.cos(angle) - 1)};
  };
  const range = distance => distance / 400;
  const heightRange = height => height / 400;

  function show2D(host, scan) {
    host.classList.add('snapshot-two');
    host.innerHTML = '<section><h3>俯视图 · 距离 / 方位</h3><canvas aria-label="保存的二维雷达"></canvas></section><section><h3>高度投影 · 相对高度 cm</h3><canvas aria-label="保存的障碍物高度"></canvas></section>';
    const canvases = [...host.querySelectorAll('canvas')];
    const draw = () => canvases.forEach((canvas, index) => {
      const rect = canvas.getBoundingClientRect(), w = Math.max(1, rect.width), h = Math.max(1, rect.height);
      const dpr = Math.min(root.devicePixelRatio || 1, 2), ctx = canvas.getContext('2d');
      canvas.width = w * dpr; canvas.height = h * dpr; ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.fillStyle = '#111a16'; ctx.fillRect(0, 0, w, h); ctx.font = '11px sans-serif';
      ctx.strokeStyle = '#304139'; ctx.fillStyle = '#93a69c'; ctx.lineWidth = 1;
      const cx = w / 2, cy = h - 32, radius = Math.min(w * .43, h * .76);
      const left = 36, top = 16, bottom = h - 28, oy = (top + bottom) / 2, pw = w - left - 15, ph = (bottom - top) / 2;
      [0, 100, 200, 300, 400].forEach(distance => {
        ctx.beginPath();
        if (!index) {
          const r = range(distance) * radius; ctx.arc(cx, cy, r, Math.PI, 0);
          ctx.fillText(distance + ' cm', cx + 4, cy - r - 4);
        } else {
          const x = left + range(distance) * pw; ctx.moveTo(x, top); ctx.lineTo(x, bottom);
          ctx.fillText(distance, x - 7, h - 8);
        }
        ctx.stroke();
      });
      (index ? [-400, -200, 0, 200, 400] : [-90, -60, -30, 0, 30, 60, 90]).forEach(value => {
        ctx.beginPath();
        if (!index) {
          const a = radians(value - 90); ctx.moveTo(cx, cy); ctx.lineTo(cx + radius * Math.cos(a), cy + radius * Math.sin(a));
        } else {
          const y = oy - heightRange(value) * ph; ctx.moveTo(left, y); ctx.lineTo(w - 15, y); ctx.fillText(value, 3, y + 4);
        }
        ctx.stroke();
      });
      scan.points.forEach(point => {
        const p = projection(point.distance, point.pitch), a = radians(-point.yaw - 90);
        const x = index ? left + range(p.forward) * pw : cx + range(p.forward) * radius * Math.cos(a);
        const y = index ? oy - heightRange(p.height) * ph : cy + range(p.forward) * radius * Math.sin(a);
        ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI * 2); ctx.fillStyle = '#' + color(point.distance).toString(16).padStart(6, '0'); ctx.fill();
      });
      ctx.beginPath(); ctx.arc(index ? left : cx, index ? oy : cy, 4, 0, Math.PI * 2); ctx.fillStyle = '#edf5f0'; ctx.fill();
    });
    const observer = new ResizeObserver(draw); observer.observe(host); draw();
    return () => observer.disconnect();
  }

  function gasGeometry(T,marks){
    const xyz=[],colors=[];
    // Draw exceptional readings last so a return over the same spot retains the alert.
    for(const alert of [false,true])for(const m of marks){
      if(['warning','alarm','fault'].includes(m.level)!==alert)continue;
      xyz.push(m.y*400,alert?9:5,m.x*400);
      new T.Color(root.RouteGas.colors[m.level]).toArray(colors,colors.length);
    }
    const g=new T.BufferGeometry();
    g.setAttribute('position',new T.Float32BufferAttribute(xyz,3));
    g.setAttribute('color',new T.Float32BufferAttribute(colors,3));
    return g;
  }
  function show3D(host, scan,route) {
    const T = root.THREE;
    if (!T || !T.OrbitControls || !root.GimbalModel || typeof root.GimbalModel.create !== 'function')
      throw Error('三维回看组件未加载，请保留同目录的 three.min.js、OrbitControls.umd.js、rover-model-data.js 和 gimbal-model.js。');
    let renderer;
    try { renderer = new T.WebGLRenderer({antialias: true}); }
    catch (_) { throw Error('无法启动三维回看，请确认浏览器支持 WebGL。'); }
    renderer.setPixelRatio(Math.min(root.devicePixelRatio || 1, 2)); renderer.setClearColor(0x0c1310, 1);
    host.append(renderer.domElement); renderer.domElement.setAttribute('aria-label', '保存的三维点云，拖动旋转、滚轮缩放');
    const scene = new T.Scene(), camera = new T.PerspectiveCamera(48, 1, .1, 1800);
    const controls = new T.OrbitControls(camera, renderer.domElement);
    camera.position.fromArray(scan.view ? scan.view.camera : [300, 220, 380]);
    controls.target.fromArray(scan.view ? scan.view.target : [0, 65, 80]);
    controls.minDistance = 120; controls.maxDistance = 900; controls.maxPolarAngle = Math.PI * .48;
    scene.add(new T.HemisphereLight(0xcfffe7, 0x07110b, 2.4), new T.GridHelper(800, 16, 0x3b7356, 0x1f3329), new T.AxesHelper(80));
    const key = new T.DirectionalLight(0xd8fff0, 1.5); key.position.set(90, 170, 120); scene.add(key);
    // Each view owns its model resources; closing a saved scan never disposes the live model.
    const {machine, pointAt} = root.GimbalModel.create(T); scene.add(machine);
    const scanBounds=new T.Box3().setFromObject(machine),fitPoints=[];
    const {min,max}=scanBounds;
    for(const x of [min.x,max.x])for(const y of [min.y,max.y])for(const z of [min.z,max.z])fitPoints.push(new T.Vector3(x,y,z));
    const positions=[],tints=[],samples=[];
    function add(point,pos,station){
      pos.toArray(positions,positions.length);new T.Color(color(point.distance)).toArray(tints,tints.length);
      scanBounds.expandByPoint(pos);fitPoints.push(pos);
      samples.push({x:pos.x/400,y:pos.y/400,z:pos.z/400,yaw:point.yaw,pitch:point.pitch,distance:point.distance,station});
    }
    function addScan(saved,pose,station){
      for(const p of saved.points){
        const pos=p.position?new T.Vector3(...p.position).multiplyScalar(400):pointAt(p.distance*4,p.yaw,p.pitch);
        if(pose)pos.applyAxisAngle(new T.Vector3(0,1,0),pose.heading).add(new T.Vector3(pose.y*400,0,pose.x*400));
        add(p,pos,p.station||station);
      }
    }
    if(route){
      if(route.worldCloud)for(const p of route.worldCloud)add(p,new T.Vector3(p.x,p.y,p.z).multiplyScalar(400),p.station);
      else for(const checkpoint of route.scans||[])if(checkpoint.scan.kind==='3d')
        addScan(checkpoint.scan,checkpoint.pose,'scan'+checkpoint.id);
      const line=[];
      route.points.forEach((p,i)=>{
        const pos=new T.Vector3(p.y*400,2,p.x*400);scanBounds.expandByPoint(pos);fitPoints.push(pos);
        if(i&&!p.gap)line.push(route.points[i-1].y*400,2,route.points[i-1].x*400,pos.x,pos.y,pos.z);
      });
      const path=new T.BufferGeometry();path.setAttribute('position',new T.Float32BufferAttribute(line,3));
      scene.add(new T.LineSegments(path,new T.LineBasicMaterial({color:0x78bfff})));
      const marks=new T.Points(gasGeometry(T,route.gasMarks||[]),
        new T.PointsMaterial({size:7,vertexColors:true,sizeAttenuation:false,depthTest:false}));
      marks.renderOrder=3;scene.add(marks);
      machine.position.set(route.y*400,0,route.x*400);machine.rotation.y=route.heading;
    }else addScan(scan,null,'snapshot');
    const cloud=new T.BufferGeometry();
    cloud.setAttribute('position',new T.Float32BufferAttribute(positions,3));
    cloud.setAttribute('color',new T.Float32BufferAttribute(tints,3));
    scene.add(new T.Points(cloud,new T.PointsMaterial({size:4,vertexColors:true,sizeAttenuation:false})));
    const walls=root.ScanWalls.layer(T,400),wallResult=walls.update(samples);
    const remaining=[];samples.forEach((p,i)=>{if(!wallResult.covered.has(p))remaining.push(i);});
    cloud.setIndex(remaining);scene.add(walls.mesh);
    const worldSnapshot=route||scan.points.some(point=>point.position);
    const render = () => renderer.render(scene, camera);
    const resize = () => {
      const rect = host.getBoundingClientRect(), w = Math.max(1, rect.width), h = Math.max(1, rect.height);
      renderer.setSize(w, h, false); camera.aspect = w / h;
      if(scan.dataSource==='simulation'||worldSnapshot||scan.points.some(point=>point.distance>150)){
        const center=scanBounds.getCenter(new T.Vector3()),direction=camera.position.clone().sub(controls.target).normalize();
        const right=new T.Vector3().crossVectors(camera.up,direction).normalize(),up=new T.Vector3().crossVectors(direction,right).normalize(),tan=Math.tan(T.MathUtils.degToRad(camera.fov/2));
        let distance=150;
        for(const point of fitPoints){const delta=point.clone().sub(center),depth=delta.dot(direction);distance=Math.max(distance,depth+Math.abs(delta.dot(up))/tan,depth+Math.abs(delta.dot(right))/(tan*camera.aspect));}
        distance*=1.1;
        controls.maxDistance=Math.max(1800,distance*1.3);controls.target.copy(center);camera.position.copy(center).addScaledVector(direction,distance);controls.update();
      }
      camera.far=Math.max(1800,controls.maxDistance+scanBounds.getSize(new T.Vector3()).length());
      camera.updateProjectionMatrix(); render();
    };
    controls.addEventListener('change', render); controls.update();
    const observer = new ResizeObserver(resize); observer.observe(host); resize();
    // A static saved scene renders only on resize/interaction; no animation loop is needed.
    return () => {
      scene.remove(walls.mesh);walls.dispose();
      observer.disconnect(); controls.removeEventListener('change', render); controls.dispose();
      const geometries = new Set(), allMaterials = new Set();
      scene.traverse(object => {
        if (object.geometry) geometries.add(object.geometry);
        if (object.material) (Array.isArray(object.material) ? object.material : [object.material]).forEach(m => allMaterials.add(m));
      });
      geometries.forEach(g => g.dispose()); allMaterials.forEach(m => m.dispose()); renderer.dispose(); renderer.forceContextLoss();
    };
  }

  function open(checkpoint,route) {
    if (dismiss) dismiss();
    const scan = checkpoint.scan;
    if (!scan || !['2d', '3d'].includes(scan.kind) || !Array.isArray(scan.points) || (!route && !scan.points.length))
      throw Error('这个路线点没有可回看的扫描快照。');
    let previousFocus = document.activeElement;
    while (previousFocus && previousFocus.shadowRoot && previousFocus.shadowRoot.activeElement) previousFocus = previousFocus.shadowRoot.activeElement;
    const dialog = document.createElement('dialog'); dialog.id = 'scan-snapshot-viewer';
    dialog.setAttribute('aria-labelledby', 'scan-snapshot-title'); dialog.setAttribute('aria-describedby', 'scan-snapshot-meta');
    dialog.innerHTML = `<style>
      #scan-snapshot-viewer{box-sizing:border-box;width:min(1050px,calc(100vw - 24px));max-height:calc(100dvh - 24px);padding:20px;border:1px solid #456251;border-radius:10px;background:#18211e;color:#edf5f0;font:14px/1.5 "Segoe UI","Microsoft YaHei",sans-serif;overflow:auto}
      #scan-snapshot-viewer::backdrop{background:rgba(0,0,0,.75)}
      #scan-snapshot-viewer header{display:flex;align-items:center;justify-content:space-between;gap:16px}
      #scan-snapshot-viewer h2{font-size:19px;margin:0}#scan-snapshot-viewer h3{font-size:13px;margin:0 0 8px;font-weight:500}
      #scan-snapshot-viewer button{font:inherit;cursor:pointer;background:#26382e;color:#edf5f0;border:1px solid #6b9a7c;border-radius:5px;padding:8px 16px;white-space:nowrap}
      #scan-snapshot-viewer button:focus-visible{outline:3px solid #48d597;outline-offset:3px}
      #scan-snapshot-viewer p{color:#a9bdb0;margin:10px 0;overflow-wrap:anywhere}
      #scan-snapshot-viewer .snapshot-view{height:min(58vh,530px);min-height:280px;border:1px solid #324b3c;border-radius:6px;overflow:hidden;background:#0c1310}
      #scan-snapshot-viewer .snapshot-view>canvas{width:100%;height:100%;display:block;touch-action:none}
      #scan-snapshot-viewer .snapshot-two{display:grid;grid-template-columns:1fr 1fr;gap:12px;padding:12px;box-sizing:border-box}
      #scan-snapshot-viewer .snapshot-two section{display:flex;flex-direction:column;min-height:0;min-width:0}
      #scan-snapshot-viewer .snapshot-two canvas{width:100%;height:100%;min-height:0;flex:1}
      #scan-snapshot-viewer .snapshot-legend{font-size:12px}
      @media(max-width:600px){#scan-snapshot-viewer{padding:13px}#scan-snapshot-viewer .snapshot-two{grid-template-columns:1fr;height:65vh;min-height:430px}#scan-snapshot-viewer h2{font-size:16px}}
    </style><header><h2 id="scan-snapshot-title"></h2><button type="button" autofocus>关闭</button></header><p id="scan-snapshot-meta"></p><div class="snapshot-view"></div><p class="snapshot-legend"></p>`;
    const pose = checkpoint.pose;
    dialog.querySelector('h2').textContent = '路线点 #' + checkpoint.id + ' · ' +
      (checkpoint.label?checkpoint.label+' · ':'')+scan.kind.toUpperCase() + ' 扫描快照';
    dialog.querySelector('#scan-snapshot-meta').textContent = new Date(checkpoint.capturedAt).toLocaleString('zh-CN', {hour12: false}) +
      ' · X ' + pose.x.toFixed(2) + ' m · Y ' + pose.y.toFixed(2) + ' m · 航向 ' + (pose.heading * 180 / Math.PI).toFixed(1) + '° · ' + scan.points.length + ' 个扫描点';
    dialog.querySelector('.snapshot-legend').textContent = '红色 <20 cm · 黄色 20–40 cm · 绿色 40–400 cm。' +
      (scan.kind === '3d' ? (scan.points.some(p=>p.position)?'图中障碍点保留采集位置。':'')+'拖动旋转，滚轮或双指缩放。' : '') +
      (scan.dataSource==='simulation'?'已保存的地貌扫描点图。':'这是保存时的只读快照。');
    if(route){
      dialog.querySelector('h2').textContent=(route.dataSource==='simulation'?'模拟路线':'巡检路线')+' · 整程三维图';
      dialog.querySelector('#scan-snapshot-meta').textContent=route.distance.toFixed(2)+' m · '+(route.gasMarks||[]).length+' 个气体采样位置';
      dialog.querySelector('.snapshot-legend').textContent='蓝线为已走路线。气体采样点：绿正常、黄预警、红报警、紫故障、灰数据不全。石纹表示拟合墙面。';
    }else if(scan.kind==='3d')dialog.querySelector('.snapshot-legend').textContent+=' 石纹表示拟合墙面。';
    root.dispatchEvent(new CustomEvent('gimbal:snapshot-open'));
    let destroy = () => {}, done = false;
    const finish = () => {
      if (done) return; done = true; destroy(); dialog.remove();
      if (dismiss === close) dismiss = null;
      if (previousFocus && previousFocus.isConnected) previousFocus.focus();
    };
    const close = () => { dialog.close(); finish(); };
    dismiss = close; dialog.querySelector('button').addEventListener('click', close); dialog.addEventListener('close', finish, {once: true});
    document.body.append(dialog); dialog.showModal();
    try { destroy = scan.kind === '3d' ? show3D(dialog.querySelector('.snapshot-view'), scan,route) : show2D(dialog.querySelector('.snapshot-view'), scan); }
    catch (error) { close(); throw error; }
    return dialog;
  }
  root.ScanSnapshotViewer = {open,gasGeometry,openMap(route){
    if(!route?.points?.length)throw Error('还没有路线可查看。');
    return open({id:0,capturedAt:route.originTime,pose:route,scan:{kind:'3d',points:[]}},route);
  }};
})(typeof globalThis !== 'undefined' ? globalThis : this);
