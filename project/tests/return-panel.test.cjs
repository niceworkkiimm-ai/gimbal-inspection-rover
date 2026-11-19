'use strict';
const assert=require('node:assert/strict');
const test=require('node:test');
const {ReturnClient,valid,acknowledged}=require('../return-panel.js');
const base=()=>({online:true,boot:913,ack:1,result:0,task:1,state:1,reason:0,points:8,index:0,ready:true,action:1,commandTask:0});
function fixture(){
  let status=base();const calls=[];
  const client=new ReturnClient(async(url,options)=>{
    const params=new URL(url).searchParams;calls.push({params,method:options.method});
    return {ok:true,json:async()=>options.method==='GET'?{...status}:{pending:true}};
  },'http://example.test/api/return');
  return {client,calls,set:s=>status=s};
}
test('validates status and matches all command ACK identity fields',()=>{
  const s=base(),cmd={boot:913,request:1,code:1,task:0};
  assert(valid(s));assert(acknowledged(s,cmd));
  for(const field of ['boot','ack','action','commandTask'])assert(!acknowledged({...s,[field]:99},cmd));
  assert(!valid({...s,points:1025}));assert(!valid({...s,state:NaN}));
});
test('HTTP accepted is not MCU execution; heartbeat starts only after matching ACK',async()=>{
  const f=fixture();await f.client.poll();await f.client.command('start');
  assert.equal(f.client.owner,null);assert(f.client.pending);
  await f.client.poll();assert(!f.calls.some(c=>c.params.get('action')==='heartbeat'));
  f.set({...base(),ack:2,state:2,action:2,commandTask:1});await f.client.poll();
  assert.equal(f.client.pending,null);assert.deepEqual(f.client.owner,{boot:913,task:1});
  assert(f.calls.some(c=>c.params.get('action')==='heartbeat'));
});
test('duplicate clicks blocked; stop can supersede pending start without old ACK reviving it',async()=>{
  const f=fixture();await f.client.poll();await f.client.command('start');
  await assert.rejects(f.client.command('start'));
  await f.client.command('stop');assert.equal(f.client.pending.request,3);
  f.set({...base(),ack:2,state:2,action:2,commandTask:1});await f.client.poll();
  assert.equal(f.client.owner,null);assert.equal(f.client.pending.code,3);
  f.set({...base(),ack:3,state:4,reason:7,action:3,commandTask:1});await f.client.poll();
  assert.equal(f.client.pending,null);assert.equal(f.client.owner,null);
});
test('reboot, hidden tab and unowned running task never acquire a heartbeat lease',async()=>{
  const f=fixture();await f.client.poll();await f.client.command('start');
  f.client.suspended=true;f.set({...base(),ack:2,state:2,action:2,commandTask:1});
  await f.client.poll();assert.equal(f.client.owner,null);
  f.client.suspended=false;await f.client.poll();assert.equal(f.client.owner,null);
  f.set({...base(),boot:914,ack:0,state:0,task:0});await f.client.poll();assert.equal(f.client.owner,null);
});
test('transport failure does not retry start and disconnection clears ownership',async()=>{
  const f=fixture();await f.client.poll();let posts=0;
  f.client.fetch=async()=>{posts++;throw new Error('offline');};
  await assert.rejects(f.client.command('start'));assert.equal(posts,1);
  f.client.disconnect();assert.equal(f.client.status,null);assert.equal(f.client.owner,null);
});
test('wheel diagnostics retain the last confirmed MCU observation when the connection goes away',async()=>{
  const f=fixture();
  assert.equal(f.client.lastObservation,null);
  f.set({...base(),state:4,reason:9});
  await f.client.poll();
  const stop=f.client.lastObservation;
  assert.equal(stop.state,'已中止');assert.equal(stop.reason,'编码器计数异常');
  assert.equal(stop.task,1);assert.equal(stop.boot,913);assert(stop.at>0);
  f.set({...base(),online:false});await f.client.poll();f.client.disconnect();
  assert.equal(f.client.lastObservation,stop);
  assert.equal(f.client.owner,null);
  f.set({...base(),boot:914,task:0,state:0,reason:0});await f.client.poll();
  assert.equal(f.client.lastObservation.boot,914);assert.equal(f.client.lastObservation.reason,'');
  assert.equal(stop.reason,'编码器计数异常');
});
