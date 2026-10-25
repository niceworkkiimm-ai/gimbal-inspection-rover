'use strict';
const assert=require('node:assert/strict');
const {makeScenario,makeScan}=require('../rover-demo.js');
const {routeRecord,parseRouteFile}=require('../vehicle-route.js');
const s=makeScenario(),origin=1792890000000;
let passed=0;
function check(name,fn){fn();passed++;console.log('PASS '+name);}
check('five deterministic terrains contain distinct, bounded scanner returns',()=>{
  assert.equal(s.scans.length,5);
  const shapes=new Set();
  s.scans.forEach((scan,i)=>{
    assert.equal(scan.dataSource,'simulation');assert(scan.points.length>=120&&scan.points.length<=800);
    for(const p of scan.points){assert(p.distance>=2&&p.distance<=150);assert(p.yaw>=-70&&p.yaw<=70);assert(p.pitch>=-32&&p.pitch<=32);}
    assert.deepEqual(scan,makeScan(i));shapes.add(JSON.stringify(scan.points));
  });
  assert.equal(shapes.size,5);
});
check('the full moving route and every scan survive normal JSON save/open with provenance',()=>{
  const route=s.routeAt(s.duration,origin),record=routeRecord(route,origin);
  const parsed=parseRouteFile(JSON.stringify(record));
  assert.equal(parsed.dataSource,'simulation');assert.equal(parsed.scans.length,5);
  assert.deepEqual(parsed,record.route);assert(route.distance>10&&route.distance<30);
  assert(Math.hypot(route.x,route.y)<.01);
  for(const checkpoint of parsed.scans){
    assert.equal(checkpoint.scan.dataSource,'simulation');assert(checkpoint.label.length>0);
    assert(checkpoint.pose.time<=parsed.elapsed);assert(checkpoint.pose.distance<=parsed.distance);
  }
});
check('each stop has zero wheel speed and no distance accumulation; moving data remains continuous',()=>{
  for(const phase of s.phases.filter(p=>p.type==='scan')){
    const a=s.sampleAt(phase.start+1),b=s.sampleAt(phase.end-1);
    assert.equal(a.speed,0);assert.deepEqual(a.rpm,[0,0,0,0]);assert(Math.abs(a.distance-b.distance)<1e-8);
    assert(Math.hypot(a.x-b.x,a.y-b.y)<1e-8);
  }
  s.samples.forEach((p,i)=>{if(!i)return;const q=s.samples[i-1];assert(p.distance>=q.distance);assert(Math.hypot(p.x-q.x,p.y-q.y)<.08);});
});
check('seeking backwards removes future checkpoints and exported snapshots own their arrays',()=>{
  const full=s.routeAt(s.duration,origin),early=s.routeAt(s.checkpoints[1].at+.1,origin),zero=s.routeAt(0,origin);
  assert.equal(early.scans.length,2);assert.equal(zero.scans.length,0);assert.equal(zero.distance,0);
  assert(early.points.length<full.points.length);
  const n=s.scans[0].points[0].distance;full.scans[0].scan.points[0].distance=999;
  assert.equal(s.scans[0].points[0].distance,n);assert.equal(early.scans[0].scan.points[0].distance,n);
});
console.log(passed+' passed, 0 failed');
