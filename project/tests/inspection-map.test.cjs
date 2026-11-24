'use strict';
const assert=require('node:assert/strict');
const {build}=require('../scan-walls.js');
const gas=require('../route-gas.js');
const {RouteEngine,defaults,routeRecord,parseRouteFile}=require('../vehicle-route.js');
function row(pitch,z=1) {
  return Array.from({length:23},(_,i)=>{
    const yaw=(i-11)*2,x=z*Math.tan(yaw*Math.PI/180);
    return {x,y:.24+Math.tan(pitch*Math.PI/180)*Math.hypot(x,z),z,yaw,pitch,
      distance:100*Math.hypot(x,z),station:'0'};
  });
}
const flat=row(0),high=row(5);
assert(build(flat).bands>0);assert.equal(build(flat).panels,0);
assert(build([...flat,...high]).panels>0);
const door=[...flat,...high].filter(p=>Math.abs(p.yaw)>4);
const mesh=build(door);
assert(mesh.panels>0);
for(let i=0;i<mesh.vertices.length;i+=18){
  const xs=[0,3,6,9,12,15].map(k=>mesh.vertices[i+k]);
  assert(Math.max(...xs)<0||Math.min(...xs)>0);
}
assert.equal(build(flat.slice(0,4)).vertices.length,0);
assert.equal(build(flat.map(p=>({...p,y:.03,pitch:-15}))).vertices.length,0);
assert.equal(build([...flat,...high.map(p=>({...p,station:'1'}))]).panels,0);
assert.equal(build([...flat,...row(5,1.6)]).panels,0);
const rows=[1,2,3,4].map(channel=>({channel,online:true,valid:true,severity:'normal',
  concentration:1,unit:'ppm',statusText:'正常',name:'CH'+channel}));
assert.equal(gas.summarize(rows).level,'normal');
assert.equal(gas.summarize(rows.slice(1)).level,'unknown');
assert.equal(gas.summarize(rows.map(g=>({...g,valid:false}))).level,'unknown');
assert.equal(gas.summarize([{...rows[0],severity:'alarm',statusText:'超量程',valid:false}]).level,'alarm');
const e=new RouteEngine({...defaults(),confirmed:true});
e.session='test';e.online=true;e.addPoint(true,true);const now=Date.now();
e.rememberPose(0,false,now);e.lastReceived=now;
gas.observe(e,gas.summarize(rows),now);gas.observe(e,gas.summarize(rows),now);
const alarm=gas.summarize([{...rows[0],severity:'alarm',statusText:'高限报警'},...rows.slice(1)]);
gas.observe(e,alarm,now);gas.observe(e,gas.summarize(rows),now);
assert.deepEqual(e.gasMarks.map(m=>m.level),['normal','alarm','normal']);
e.logPending=true;gas.observe(e,alarm,now);e.logPending=false;gas.observe(e,alarm,now+1300);
assert.equal(e.gasMarks.length,3);
const file=routeRecord({...e,worldCloud:flat});
const parsed=parseRouteFile(JSON.stringify(file));
assert.equal(parsed.gasMarks[1].level,'alarm');assert.deepEqual(parsed.worldCloud,flat);
delete file.route.gasMarks;delete file.route.worldCloud;
assert.equal(parseRouteFile(JSON.stringify(file)).gasMarks,undefined);
e.reset();assert.equal(e.gasMarks.length,0);
console.log('PASS wall bands, multiple heights, doorway, floor, isolated points, pose and depth boundaries');
console.log('PASS gas availability, alarm priority, stationary recovery, stale positions, JSON and legacy files');
