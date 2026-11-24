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

