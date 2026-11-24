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

// Stagger every other sweep. Remove beams, add bounded range noise and an isolated spike.
function measured(yaw,pitch,z=1) {
  const x=z*Math.tan(yaw*Math.PI/180),d=Math.hypot(x,z);
  return {x,y:.34+d*Math.tan(pitch*Math.PI/180),z,yaw,pitch,distance:100*d,station:'fixed'};
}
const sparse=[],spike=measured(0,0,2.1);
for(const pitch of [-10,-5,0,5,10]){
  for(let i=-12;i<=12;i++){
    const yaw=2*i+(pitch%10===0?0:1);
    if((i+2*pitch)%13===4)continue;
    sparse.push(yaw===0&&pitch===0?spike:measured(yaw,pitch,1+.008*Math.sin(i*1.7+pitch)));
  }
}
const joined=build(sparse);
assert(!joined.covered.has(spike));assert(joined.covered.size>=sparse.length*.88);
assert(joined.vertices.every((v,i)=>i%3!==2||Math.abs(v-1)<.012));
function containsXZHeight(mesh,x,y){
  // Cast a ray through the wall at a known horizontal position and height.
  const v=mesh.vertices;
  for(let i=0;i<v.length;i+=9){
    const [ax,ay,,bx,by,,cx,cy]=v.slice(i,i+9);
    const cross=(px,py,qx,qy)=>px*qy-py*qx;
    const d=cross(bx-ax,by-ay,cx-ax,cy-ay);if(Math.abs(d)<1e-10)continue;
    const u=cross(x-ax,y-ay,cx-ax,cy-ay)/d,w=cross(bx-ax,by-ay,x-ax,y-ay)/d;
    if(u>=-1e-6&&w>=-1e-6&&u+w<=1.000001)return true;
  }
  return false;
}
for(let x=-.32;x<=.32;x+=.04)for(let h=.2;h<=.48;h+=.025)
  assert(containsXZHeight(joined,x,h),'hole inside supported noisy wall '+[x,h]);
assert.deepEqual(build([...sparse].reverse()).vertices,joined.vertices);
const edgesAt=(pitch,depth)=>Array.from({length:21},(_,i)=>measured((i-10)*2,pitch,depth));
const below=edgesAt(0,1),above=edgesAt(10,1);
assert(containsXZHeight(build([...below,...above]),0,.42));
const blocked=build([...below,measured(0,5,1.5),...above]);
assert(!containsXZHeight(blocked,0,.42));assert(containsXZHeight(blocked,.15,.42));
assert(!containsXZHeight(build([...below,...edgesAt(15,1)]),0,.42));
const opening=sparse.filter(p=>Math.abs(p.yaw)>7);
assert(!containsXZHeight(build(opening),0,.31));
const stepped=[...edgesAt(0,1),...edgesAt(5,1)].map(p=>p.yaw>=0?measured(p.yaw,p.pitch,1.3):p);
const stepSurface=build(stepped);
for(let i=0;i<stepSurface.vertices.length;i+=9){
 const z=[2,5,8].map(k=>stepSurface.vertices[i+k]);
 assert(Math.max(...z)-Math.min(...z)<.02,'rectangle spans a depth step');
}
// Two planar returns meeting at a real convex corner must retain the kink.
const corner=[...edgesAt(0,1),...edgesAt(5,1)].map(p=>{
 const slope=p.yaw<0?-.75:.75;
 return measured(p.yaw,p.pitch,1/(1-slope*Math.tan(p.yaw*Math.PI/180)));
});
const bent=build(corner);assert(bent.panels>0);
for(let i=0;i<bent.vertices.length;i+=3){
 const x=bent.vertices[i],z=bent.vertices[i+2];
 assert(Math.abs(z-(1+.75*Math.abs(x)))<.015);
}
console.log('PASS staggered noisy wall coverage, spike rejection, door and corner, missing row and depth veto');
