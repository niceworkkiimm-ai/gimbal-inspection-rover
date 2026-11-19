/* V8.22: physical return controls never read saved routes or demo samples. */
(function(root) {
  'use strict';
  const states=['未设起点','正在记录','倒车返程中','返程完成','已中止'];
  const reasons=['','遥控失联','手动接管或模式改变','轮速或控制故障','记录采样中断',
    '记录容量已满','返程超时','主动停止','网页心跳中断','编码器计数异常'];
  const results=['已执行','请切到驾驶模式，回中并停车半秒','任务已变化，请重新设起点',
    '没有完整可用的返程记录','主控已重启','命令过期','正在返程，不能重设起点'];
  const uint=n=>Number.isInteger(n)&&n>=0&&n<=4294967295;
  function valid(s) {
    return s&&['boot','ack','result','task','state','reason','points','index','action','commandTask'].every(k=>uint(s[k]))&&
      s.state<5&&s.reason<10&&s.result<7&&s.points<=1024&&s.index<1024&&s.action<=3&&
      typeof s.online==='boolean'&&typeof s.ready==='boolean';
  }
  function acknowledged(s,c) {
    return valid(s)&&s.online&&s.boot===c.boot&&s.ack===c.request&&s.action===c.code&&s.commandTask===c.task;
  }
  class ReturnClient {
    constructor(fetcher,base) {
      this.fetch=fetcher;this.base=base;this.status=null;this.pending=null;
      this.lastObservation=null;
      this.owner=null;this.pulse=0;this.message='等待返程固件';this.suspended=false;
    }
    async request(params,method='GET') {
      const controller=new AbortController();const timer=setTimeout(()=>controller.abort(),600);
      try {
        const response=await this.fetch(this.base+'?'+new URLSearchParams(params),
          {method,cache:'no-store',signal:controller.signal});
        if(!response.ok)throw new Error('HTTP '+response.status);
        return await response.json();
      } finally {clearTimeout(timer);}
    }
    async poll() {
      const s=await this.request({t:Date.now()});
      if(!valid(s))throw new Error('无效返程状态');
      this.status=s;
      if(!s.online){this.message='返程主控离线';this.owner=null;return;}
      if(s.boot){
        this.lastObservation={
          at:Date.now(),boot:s.boot,task:s.task,state:states[s.state],reason:reasons[s.reason],
          points:s.points,index:s.index
        };
      }
      if(['等待返程固件','返程主控离线','连接中断；返程心跳已停止'].includes(this.message))this.message='主控状态已同步';
      if(this.pending&&acknowledged(s,this.pending)) {
        const c=this.pending;this.pending=null;this.message=results[s.result];
        if(!this.suspended&&s.result===0&&c.code===2&&s.state===2){this.owner={boot:s.boot,task:s.task};this.pulse=0;}
      } else if(this.pending&&(s.boot!==this.pending.boot||
          ((s.ack-this.pending.request)>>>0)<2147483648&&s.ack!==this.pending.request)) {
        this.pending=null;this.owner=null;this.message='任务或命令已变化，请查看主控状态';
      }
      if(this.pending&&Date.now()-this.pending.at>1800){
        this.pending=null;this.owner=null;this.message='未收到主控确认；请核对状态，必要时按停止';
      }
      if(this.owner&&(s.state!==2||s.boot!==this.owner.boot||s.task!==this.owner.task))this.owner=null;
      if(this.owner&&!this.suspended)await this.request({action:'heartbeat',boot:s.boot,task:s.task,pulse:++this.pulse},'POST');
    }
    async command(action) {
      const s=this.status,code={set:1,start:2,stop:3}[action];
      if(!s||!s.online||!s.boot||!code)throw new Error('主控未就绪');
      if(action!=='stop'&&this.pending)throw new Error('正在等待主控确认');
      const prior=this.pending?this.pending.request:s.ack;
      const request=(prior+1)>>>0;if(!request)throw new Error('命令序号已用尽，请重启设备');
      const c={boot:s.boot,request,task:s.task,code,action,at:Date.now()};
      this.pending=c;this.owner=null;this.message='命令已提交，等待主控确认';
      /* No motion-command retries: a lost ACK must not silently start a second task. */
      await this.request({action,boot:c.boot,request:c.request,task:c.task},'POST');
    }
    disconnect() {this.status=null;this.owner=null;this.message='连接中断；返程心跳已停止';}
  }
  if(typeof module==='object'&&module.exports)module.exports={ReturnClient,valid,acknowledged};
  if(!root.document)return;
  function mount() {
    const demo=new URLSearchParams(root.location.search).get('demo')==='1'||
      /模拟演示/.test(decodeURI(root.location.pathname));
    const panel=document.createElement('section');panel.className='return-panel';
    panel.innerHTML='<strong>原路返回 · 试验版</strong><div class="return-actions">'+
      '<button type="button" data-return="set">设为返程起点</button>'+
      '<button type="button" data-return="start">倒车原路返回</button>'+
      '<button type="button" data-return="stop">停止</button></div>'+
      '<p data-return-state role="status">等待返程固件</p><p data-return-detail></p>'+
      '<small>先在驾驶模式回中停车，再设起点。返程时保持手持回中和本页前台运行；移动摇杆或倾斜手持可中止。'+
      '后方没有障碍检测，请现场看护。完成状态只表示编码器目标到达，打滑会产生位置误差。</small>';
    const style=document.createElement('style');style.textContent='.return-panel{margin:16px auto;padding:18px;max-width:1500px;border:1px solid #78909c55;border-radius:12px;background:#f7fbff;color:#19354a;font:14px/1.5 system-ui}.return-actions{display:flex;flex-wrap:wrap;gap:10px;margin:12px 0}.return-actions button{border:1px solid #78909c;padding:9px 14px;border-radius:7px;cursor:pointer;background:white;color:#19354a}.return-actions button[data-return=stop]{background:#b92b35;color:white}.return-actions button:disabled{opacity:.45;cursor:not-allowed}.return-panel p{margin:6px 0}.return-panel small{color:#52667a}';
    document.head.appendChild(style);
    const host=document.querySelector('main')||document.body;host.appendChild(panel);
    const output=panel.querySelector('[data-return-state]'),detail=panel.querySelector('[data-return-detail]');
    const buttons=[...panel.querySelectorAll('button')];
    if(demo){output.textContent='演示/回放模式：实车返程控制已禁用';buttons.forEach(b=>b.disabled=true);return;}
    const client=new ReturnClient(root.fetch.bind(root),'http://192.168.4.1/api/return');
    root.readReturnObservation=()=>client.lastObservation;
    let busy=false,timer;
    function render() {
      const s=client.status,live=s&&s.online&&s.boot;
      output.textContent=live?states[s.state]+(s.reason?'：'+reasons[s.reason]:''):client.message;
      detail.textContent=live?'记录 '+s.points+'/1024 点'+(s.state===2?'，剩余 '+(s.index+1)+' 个检查点':'')+' · '+client.message+
        (!s.ready&&s.state!==2?' · 请在驾驶模式回中停车半秒':''):'';
      buttons.forEach(b=>{
        const action=b.dataset.return;
        b.disabled=!live||(action!=='stop'&&(!!client.pending||!s.ready||s.state===2))||
          (action==='start'&&(s.state!==1||s.points<2));
      });
    }
    buttons.forEach(b=>b.addEventListener('click',async()=>{
      const action=b.dataset.return;
      if(action==='start'&&!root.confirm('确认后方通道畅通、有人看护，并保持遥控器回中。开始倒车返程？'))return;
      try{await client.command(action);}catch(e){client.message=e.message;}
      render();
    }));
    async function poll() {
      clearTimeout(timer);if(busy||document.hidden)return;busy=true;
      try{await client.poll();}catch(e){client.disconnect();}
      finally{busy=false;render();if(!document.hidden)timer=setTimeout(poll,300);}
    }
    document.addEventListener('visibilitychange',()=>{
      client.suspended=document.hidden;
      if(document.hidden){clearTimeout(timer);client.owner=null;client.pending=null;}else poll();
    });
    root.addEventListener('pagehide',()=>{clearTimeout(timer);client.owner=null;});
    render();poll();
  }
  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',mount);else mount();
})(typeof window==='object'?window:globalThis);
