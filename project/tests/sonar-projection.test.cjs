'use strict';
const assert=require('node:assert/strict');
const T=require('../three.min.js');
require('../rover-model-data.js');
const {pointAt}=require('../gimbal-model.js').create(T);
const {resolve,angles,nominal,validCalibration}=require('../sonar-projection.js');
const {defaults,RouteEngine,routeRecord,parseRouteFile}=require('../vehicle-route.js');
const rad=Math.PI/180;
const axis=new T.Vector3(0,1,0);
function sweep(read,{head=0,wallNormal=0,frontWall=false}={}){
  const points=[],n=new T.Vector3(Math.sin(head+wallNormal*rad),0,Math.cos(head+wallNormal*rad));
  for(const pitch of [-10,-5,0,5,10])for(let yaw=-48;yaw<=48;yaw+=2){
    const origin=pointAt(0,yaw,pitch).multiplyScalar(.0025).applyAxisAngle(axis,head);
    const dir=pointAt(4,yaw,pitch).sub(pointAt(0,yaw,pitch)).normalize().applyAxisAngle(axis,head);
    const angle=Math.acos(n.dot(dir)),near=1-origin.dot(n),r=read(near,angle,yaw,pitch,origin,dir);
    const p=origin.addScaledVector(dir,r);
    points.push({x:p.x,y:p.y,z:p.z,head,yaw,pitch,distance:r*100,station:'0',frontWall});
  }
  return {points,n};
}
const dot=(p,n)=>p.x*n.x+p.z*n.z;
const ideal=sweep((near,a)=>near/Math.cos(a));
const rayResult=resolve(ideal.points);assert.equal(rayResult.corrected,0);
assert.equal(rayResult.uncertain.size,0);
assert(ideal.points.every(p=>Math.abs(p.z-1)<1e-8));
const wide=(near,a)=>near/Math.cos(Math.max(0,a-8*rad));
for(const options of [{},{head:2.12},{head:-1.1,wallNormal:12}]){
 const {points,n}=sweep(wide,options);
 const before=JSON.stringify(points),result=resolve(points);
 assert(result.corrected>points.length*.85);
 const corrected=result.display.filter((p,i)=>p!==points[i]);
 const error=Math.max(...corrected.map(p=>Math.abs(dot(p,n)-1)));
 assert(error<.012,'estimated plane error '+error);
 assert.equal(before,JSON.stringify(points));
 assert.deepEqual(result.display.map(p=>p.distance),points.map(p=>p.distance));
 console.log('PASS synthetic 1 m plane, world heading '+(options.head||0)+', worst estimated depth error '+(100*error).toFixed(1)+' cm');
}
const narrow=ideal.points.filter(p=>Math.abs(p.yaw)<12);
assert.equal(resolve(narrow).corrected,0);
// Same equidistant data could arise on a curved wall: do not automatically turn it into a plane.
const ring=sweep(()=>1);
const ambiguous=resolve(ring.points);assert.equal(ambiguous.corrected,0);assert(ambiguous.uncertain.size>0);
assert.deepEqual(ambiguous.display,ring.points);
const unr=resolve(ring.points,{raw:true});assert.equal(unr.uncertain.size,0);assert.deepEqual(unr.display,ring.points);
const confirmed=resolve(ring.points.map(p=>({...p,frontWall:true})));
assert(confirmed.corrected>0&&confirmed.uncertain.size>0);
const different=sweep((near,a,yaw)=>yaw<0?.5:1.6);
assert.equal(resolve(different.points).corrected,0);
const off=sweep((near,a,yaw)=>yaw===32?.36:wide(near,a));
const obstacle=resolve(off.points).display.find(p=>p.yaw===32&&p.pitch===0);assert.equal(obstacle.distance,36);
assert.equal(obstacle,off.points.find(p=>p.yaw===32&&p.pitch===0));
const floor=ring.points.map(p=>({...p,y:.01}));assert.equal(resolve(floor).uncertain.size,0);
assert.deepEqual(angles(30,30,nominal),{yaw:30,pitch:30});
assert.deepEqual(angles(30,-30,[-18,18,-25,25]),{yaw:18,pitch:-25});
assert.deepEqual(angles(200,-200),{yaw:90,pitch:-90});
assert(!validCalibration([0,0,0,0]));
const e=new RouteEngine({...defaults(),confirmed:true});e.session='test';e.addPoint(true,true);
const saved={...ring.points[0],commands:[-48,-10],frontWall:true};
const file=routeRecord({...e,worldCloud:[saved]});const loaded=parseRouteFile(JSON.stringify(file));
assert.deepEqual(loaded.worldCloud,[saved]);
assert.deepEqual(resolve(ring.points.map(p=>({...p,head:undefined}))).corrected,0);
console.log('PASS equal-range ambiguity, confirmed-wall limits, foreground, tilted beams, calibration, archive');
// Convex cylinder whose centre is 1.6 m in front, radius 0.7 m. Retain its varying curvature.
const curved=sweep((near,a,yaw,pitch,o,dir)=>{
 const dx=dir.x,dz=dir.z,ox=o.x,oz=o.z-1.6,A=dx*dx+dz*dz,B=ox*dx+oz*dz,C=ox*ox+oz*oz-.49;
 const disc=B*B-A*C;return disc<0?NaN:(-B-Math.sqrt(disc))/A;
}).points.filter(p=>Number.isFinite(p.distance));
const curveResult=resolve(curved);assert.equal(curveResult.corrected,0);assert.equal(curveResult.uncertain.size,0);
const sideRing=sweep(()=>.8).points.filter(p=>p.yaw<=-2&&p.pitch===0);
assert(resolve(sideRing).uncertain.size>0);
const radians=Math.PI/180, actual=angles(30,0,[-18,18,-30,30]).yaw;
const observed=100/Math.cos(18*radians);
assert(Math.abs(observed*Math.cos(30*radians)-100)>8);
assert(Math.abs(observed*Math.cos(actual*radians)-100)<1e-8);
console.log('PASS convex cylinder, one-sided plateau and physical angle scale');


// Rounded row medians from the November 24 front sector, with the range jump above +10 degrees.
// Use generated poses and bearings so this regression contains no recorded route coordinates.
const rowsCm=[39.8,39.6,40.4,40.5,41,42.4,43.5,43.7,45,74.4,58.3,92.5,92.6];
function pitchRaster(profile,bearings=[-5,0,5],heading=1.8){
  return bearings.flatMap(yaw=>profile.map((distance,i)=>{
    const pitch=i*5-30,pos=pointAt(distance*4,yaw,pitch).multiplyScalar(.0025).applyAxisAngle(axis,heading);
    return {x:pos.x+12,y:pos.y,z:pos.z-8,head:heading,station:'pitch-test',yaw,pitch,distance};
  }));
}
const recorded=pitchRaster(rowsCm),before=JSON.stringify(recorded),projected=resolve(recorded);
assert(projected.pitchCorrected>=24);
assert.equal(projected.corrected,projected.pitchCorrected);
assert(projected.display.every((q,i)=>q.distance===recorded[i].distance&&(q.pitch<15||q===recorded[i])));
function sectionBow(points){
  const line=points.filter(p=>p.yaw===0&&p.pitch<=10);
  const depth=p=>p.x*Math.sin(p.head)+p.z*Math.cos(p.head);
  const low=line[0],high=line.at(-1);
  return Math.max(...line.map(p=>Math.abs(depth(p)-depth(low)-(depth(high)-depth(low))*(p.y-low.y)/(high.y-low.y))));
}
assert(sectionBow(recorded)>.012);
assert(sectionBow(projected.display)<.001);
const upright=resolve(recorded.map(p=>({...p,frontWall:true})));
assert(upright.pitchCorrected>0);
const near=upright.display.filter(p=>p.yaw===0&&p.pitch<=10);
const depth=p=>p.x*Math.sin(p.head)+p.z*Math.cos(p.head);
assert(Math.max(...near.map(depth))-Math.min(...near.map(depth))<.001);
assert.equal(JSON.stringify(recorded),before);
assert.equal(resolve(recorded,{raw:true}).pitchCorrected,0);
assert(resolve(pitchRaster(rowsCm,[0])).pitchCorrected>0,'vertical-only sweep');
assert(resolve(pitchRaster(rowsCm,[0]).map(p=>({...p,frontWall:true}))).pitchCorrected>0);
assert.equal(resolve(pitchRaster(rowsCm.map(()=>45))).pitchCorrected,0,'constant-radius arc');
assert(resolve(pitchRaster(rowsCm.map(()=>45),[0])).uncertain.size>0,'ambiguous pitch-only arc stays available in raw mode');
const tilted=recorded.map(p=>{
  const orig=pointAt(0,p.yaw,p.pitch).multiplyScalar(.0025).applyAxisAngle(axis,p.head);
  const dir=new T.Vector3(Math.sin(p.head+p.yaw*rad)*Math.cos(p.pitch*rad),Math.sin(p.pitch*rad),Math.cos(p.head+p.yaw*rad)*Math.cos(p.pitch*rad));
  const normal=new T.Vector3(Math.sin(p.head),-.24,Math.cos(p.head));
  const r=(.6-orig.dot(normal))/dir.dot(normal),pos=orig.addScaledVector(dir,r);
  return {...p,x:pos.x,y:pos.y,z:pos.z,distance:r*100};
});
assert.equal(resolve(tilted).pitchCorrected,0,'flat but inclined surface');
assert.equal(resolve(recorded.map(p=>({...p,y:.02}))).pitchCorrected,0,'floor');
assert.equal(resolve(recorded.filter(p=>p.pitch>-12&&p.pitch<12)).pitchCorrected,0,'sparse vertical coverage');
assert.equal(resolve(recorded.map(p=>({...p,x:p.x+.15*(p.pitch/30)}))).pitchCorrected,0,'moving station');
const mixed=[...recorded,{...recorded[3],distance:19,x:recorded[3].x-.1}];
assert.equal(resolve(mixed).display.at(-1),mixed.at(-1),'foreground return rejected');
console.log('PASS vertical raster, acquired range break, known front, vertical-only sweep, incline, radius, motion and raw view');

