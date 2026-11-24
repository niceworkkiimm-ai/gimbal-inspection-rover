/* Gas readings belong to observation locations, never to unobserved stretches of route. */
(function(root){
  'use strict';
  const colors={normal:'#c4a2ff',warning:'#ffda6a',alarm:'#ff459c',fault:'#539bff',unknown:'#9ca7b4'};
  const labels={normal:'四路正常',warning:'气体预警',alarm:'气体报警',fault:'模块故障',unknown:'数据不全'};
  const spacing=.06,MAX_MARKS=10000;
  function summarize(rows) {
    const rank={normal:0,unknown:1,fault:2,warning:3,alarm:4};
    let level='normal';const issues=[],codes=[],seen=new Set();
    for(const g of Array.isArray(rows)?rows:[]) {
      if(!g||!Number.isInteger(g.channel)||g.channel<1||g.channel>4||seen.has(g.channel))continue;
      seen.add(g.channel);
      let state='unknown';
      if(g.online===true && typeof g.statusText==='string' && typeof g.unit==='string') {
        if(g.severity==='alarm')state='alarm'; // includes sensor-reported over-range
        else if(g.severity==='fault')state='fault';
        else if(g.valid===true && Number.isFinite(g.concentration) && ['normal','warning'].includes(g.severity))state=g.severity;
      }
      if(rank[state]>rank[level])level=state;
      if(state!=='normal'){
        const name=String(g.name||('通道 '+g.channel)).slice(0,24);
        const value=g.valid===true&&g.online===true&&Number.isFinite(g.concentration)?' '+Number(g.concentration.toFixed(3))+' '+String(g.unit||'').slice(0,12):'';
        issues.push(name+' '+(g.online===true?String(g.statusText||labels[state]).slice(0,22):'离线')+value);
        codes.push(g.channel+'-'+state+'-'+(Number.isInteger(g.status)?g.status:''));
      }
    }
    if(seen.size!==4){issues.push('收到 '+seen.size+'/4 个通道');codes.push('missing');if(level==='normal')level='unknown';}
    return {level,key:codes.sort().join(','),detail:issues.join(' · ').slice(0,320)};
  }
  function observe(route,observation,now=Date.now()) {
    const pose=route.poseHistory.at(-1);
    if(!route.config.confirmed || !route.online || route.logPending || !pose || pose.gap ||
      now-route.lastReceived>1200 || now-route.poseReceivedAt>1200)return;
    const marks=route.gasMarks,last=marks.at(-1);
    if(last && last.level===observation.level && route.gasKey===observation.key &&
        last.session===route.session && route.distance-last.distance<spacing)return;
    marks.push({x:route.x,y:route.y,time:route.elapsed,distance:route.distance,
      level:observation.level,detail:observation.detail,session:route.session});
    route.gasKey=observation.key;
    if(marks.length>MAX_MARKS){marks.shift();route.gasDropped++;}
  }
  function clean(marks,route) {
    if(!Array.isArray(marks)||marks.length>MAX_MARKS)throw Error('路线气体记录数量无效。');
    let time=0,distance=0;
    return marks.map(m=>{
      if(!m||!Number.isFinite(m.x)||Math.abs(m.x)>1e6||!Number.isFinite(m.y)||Math.abs(m.y)>1e6||
        !Number.isFinite(m.time)||m.time<time||m.time>route.elapsed||!Number.isFinite(m.distance)||
        m.distance<distance||m.distance>route.distance||!Object.hasOwn(colors,m.level)||
        typeof m.detail!=='string'||m.detail.length>320)throw Error('路线气体记录格式无效。');
      time=m.time;distance=m.distance;
      return {x:m.x,y:m.y,time,distance,level:m.level,detail:m.detail};
    });
  }
  const api={spacing,colors,labels,summarize,observe,clean};
  root.RouteGas=api;if(typeof module==='object'&&module.exports)module.exports=api;
})(typeof globalThis==='undefined'?this:globalThis);
