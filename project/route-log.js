/* V8.9 / 2026-09-15. Read-only route retrieval retained; auto-drive polling branch removed.
 * Browser cursor acknowledges only fully validated batches. Missing records
 * are fetched again; overwritten records remain explicit route breaks.
 */
(function(root){
  'use strict';
  const angle=a=>Math.atan2(Math.sin(a),Math.cos(a));
  const integer=(v,a=0,b=4294967295)=>Number.isInteger(v)&&v>=a&&v<=b;
  function crc(s){let c=65535;for(const ch of s){c^=ch.charCodeAt(0)<<8;for(let i=0;i<8;i++)c=((c&32768)?(c<<1)^0x1021:c<<1)&65535;}return c;}
  const CONFIG_CRC=crc('374,97,202,210,10');
  function validate(p){
    if(!p||p.protocol!==2||typeof p.session!=='string'||!/^\d{1,10}-\d{1,10}$/.test(p.session)||
       !integer(p.oldest,1)||!integer(p.newest,p.oldest)||p.newest-p.oldest>=1024||
       p.configCrc!==CONFIG_CRC||!Array.isArray(p.samples)||p.samples.length>8)
      throw Error('车端路线数据或标定版本不匹配，请检查STM32和基座固件。');
    let previous=null;
    for(const s of p.samples){
      if(!Array.isArray(s)||s.length!==7||!integer(s[0],p.oldest,p.newest)||!integer(s[1])||
         !integer(s[2],-1e8,1e8)||!integer(s[3],-1e8,1e8)||!integer(s[4],-3142,3142)||
         !integer(s[5],0,1e9)||!integer(s[6],0,1)||
         (previous&&(s[0]!==previous[0]+1||((s[1]-previous[1])>>>0)>=2147483648||
          s[1]===previous[1]||s[5]<previous[5])))
        throw Error('路线批次不完整或顺序异常，正在重新获取。');
      previous=s;
    }
    return p;
  }
  class Reader{
    constructor(engine,changed=()=>{}){this.engine=engine;this.changed=changed;this.reset();}
    reset(){
      this.session=null;this.cursor=null;this.previous=null;this.origin=null;
      this.base={x:0,y:0,h:0,d:0,t:0};this.missed=0;this.invalidSamples=0;
      this.engine.logPending=true;this.engine.logBacklog=0;
      this.engine.logMessage='等待车端路线记录';this.changed();
    }
    ingest(input,now=Date.now()){
      const p=validate(input),e=this.engine,c=e.config;
      if(!c||!c.confirmed||c.cpr!==374||c.diameter!==97||c.track!==202||c.wheelbase!==210||
         c.signs.join(',')!=='1,-1,1,-1')
        throw Error('请确认车端标定：374次/圈，97、202、210mm，轮向 + - + -；改标定需同步修改主控。');
      const changed=this.session!==null&&this.session!==p.session;
      let samples=p.samples.filter(s=>this.session!==p.session||this.cursor===null||s[0]>this.cursor);
      if(!samples.length&&this.session!==p.session){
        this.cursor=null;e.logPending=true;this.changed();return;
      }
      if(this.cursor===null&&samples.length)samples=[samples[samples.length-1]];
      // Validate against the prior acknowledged sample before mutating any pose.
      if(!changed&&this.previous&&samples.length){
        const s=samples[0],dt=(s[1]-this.previous[1])>>>0;
        if(!dt||dt>=2147483648||s[5]<this.previous[5])
          throw Error('车端时间或累计路程异常，未接纳这一批路线。');
      }
      if(changed){
        e.restarts++;this.origin=null;this.previous=null;
        this.base={x:e.x,y:e.y,h:e.heading,d:e.distance,t:e.elapsed};
      }
      for(const s of samples){
        let gap=false;
        if(!this.origin){
          this.origin=s.slice();e.session=p.session;
          gap=changed;
        }else if(s[0]!==this.cursor+1){
          gap=true;e.gaps++;this.missed+=Math.max(0,s[0]-this.cursor-1);
        }
        if(s[6]){this.invalidSamples++;if(!gap)e.gaps++;gap=true;}
        const o=this.origin,rotation=this.base.h-o[4]/1000,co=Math.cos(rotation),si=Math.sin(rotation);
        const dx=(s[2]-o[2])/1000,dy=(s[3]-o[3])/1000;
        e.x=this.base.x+co*dx-si*dy;e.y=this.base.y+si*dx+co*dy;
        e.heading=angle(this.base.h+(s[4]-o[4])/1000);
        if(this.previous){
          const dt=(s[1]-this.previous[1])>>>0;
          e.elapsed+=dt;e.speed=gap?0:(s[5]-this.previous[5])/dt;
        }
        e.distance=this.base.d+(s[5]-o[5])/1000;
        e.addPoint(gap||!this.previous,gap||!this.previous);
        this.previous=s.slice();this.cursor=s[0];this.session=p.session;
      }
      e.logBacklog=Math.max(0,p.newest-(this.cursor||0));
      e.logPending=e.logBacklog>0;e.online=true;e.lastReceived=now;
      e.logMessage=e.logPending?'正在补传路线，还剩 '+e.logBacklog+' 帧':'车端路线已同步';
      if(this.missed)e.logMessage+='；缓存覆盖了 '+this.missed+' 帧，缺口已保留';
      if(this.invalidSamples)e.logMessage+='；主控标记 '+this.invalidSamples+' 段计数异常/采样中断，已跳过且未补造路程';
      this.changed();
    }
  }
  class Client{
    constructor(engine,{request=root.fetch.bind(root),changed=()=>{},fallback=()=>{},base='http://192.168.4.1'}={}){
      this.engine=engine;this.request=request;this.changed=changed;this.fallback=fallback;this.base=base;
      this.reader=new Reader(engine,changed);this.generation=0;this.stopped=false;this.busy=false;
    }
    reset(){this.generation++;this.reader.reset();}
    async poll(){
      if(this.busy||this.stopped)return 120;
      if(typeof document!=='undefined'&&document.hidden)return 500;
      this.busy=true;const generation=this.generation,ac=new AbortController(),timer=setTimeout(()=>ac.abort(),650);
      try{
        const after=this.reader.cursor===null?4294967295:this.reader.cursor;
        const r=await this.request(this.base+'/api/route-log?after='+after,{cache:'no-store',signal:ac.signal});
        if(generation!==this.generation)return 0;
        if(r.status===404){this.stopped=true;this.fallback();return 1000;}
        if(r.status===202)return 80;
        if(!r.ok)throw Error('路线补传暂未完成，正在重试');
        const p=await r.json();if(generation!==this.generation)return 0;
        this.reader.ingest(p);
        return this.engine.logPending?80:200;
      }catch(error){
        if(generation===this.generation){
          this.engine.online=false;this.engine.logPending=true;
          this.engine.logMessage=error.name==='AbortError'||error instanceof TypeError?
            '路线连接中断，恢复后自动补传':error.message;this.changed();
        }
        return 250;
      }finally{clearTimeout(timer);this.busy=false;}
    }
    start(){const run=async()=>{if(this.stopped)return;const delay=await this.poll();if(!this.stopped)this.timer=setTimeout(run,delay);};run();}
  }
  const api={Reader,Client,validate,CONFIG_CRC};
  if(typeof module==='object'&&module.exports)module.exports=api;
  root.RouteLogCore=api;
  if(root.VehicleRoute&&typeof document!=='undefined'&&!(root.RoverDemo&&root.RoverDemo.enabled)){
    const v=root.VehicleRoute;v.enableLog();
    const client=new Client(v.engine,{changed:v.refresh,fallback:v.legacyLog});
    root.RouteLogClient=client;client.start();
  }
})(typeof globalThis!=='undefined'?globalThis:this);
