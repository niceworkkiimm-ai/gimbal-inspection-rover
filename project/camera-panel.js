/* 项目 V8.5 / 摄像头 V1.2 / 网页协议1.0 / 2026-09-13
 * V8.5新增10帧选项，摄像头负责上下翻转，网页不重复翻转；遥测/控制不变。
 * 两页共用的独立图传窗口。只向摄像头请求JPEG，不发车辆/云台指令。
 * 默认关闭、单请求、1.2秒超时、旧帧清空、后台自动关闭；不改遥测轮询。
 */
(() => {
  'use strict';
  if(window.RoverDemo&&window.RoverDemo.enabled)return;
  const main = document.querySelector('main.app');
  if (!main || document.getElementById('camera-panel')) return;
  const host = document.createElement('section');
  host.id = 'camera-panel';
  const ui = host.attachShadow({mode:'open'});
  ui.innerHTML = `
    <style>
    :host{display:block;margin:18px 0 0;color:#ecf4ef;font:14px/1.5 "Segoe UI","Microsoft YaHei",sans-serif}
    *{box-sizing:border-box}[hidden]{display:none!important}
    .card{background:#15211b;border:1px solid #354c3e;border-radius:10px;overflow:hidden}
    .head{display:flex;align-items:center;gap:10px;flex-wrap:wrap;padding:14px 16px}
    .title{font-size:15px;white-space:nowrap}.dot{width:8px;height:8px;background:#82938a;border-radius:50%;display:inline-block;margin-right:8px}
    .dot[data-state=live]{background:#54da9d;box-shadow:0 0 10px #54da9d77}.dot[data-state=error]{background:#f2bb61}
    input,select,button{font:inherit;border:1px solid #3a5043;border-radius:6px;color:inherit;background:#1c3025;padding:7px 10px}
    input{width:235px;max-width:100%;font-size:12px}label{color:#a8bcb0;font-size:12px}button{cursor:pointer;white-space:nowrap}
    button:hover{border-color:#72c89b}button:disabled{cursor:default;opacity:.5}.primary{background:#51d699;border-color:#51d699;color:#10251a;font-weight:600}
    .actions{display:flex;gap:7px;flex-wrap:wrap;margin-left:auto}.settings{display:flex;align-items:center;gap:8px;flex-wrap:wrap}
    .body{display:grid;grid-template-columns:minmax(260px,480px) minmax(180px,1fr);gap:20px;padding:0 16px 16px}
    .view{background:#080f0b;position:relative;aspect-ratio:4/3;border:1px solid #354c3e;border-radius:7px;overflow:hidden;display:grid;place-items:center}
    .view img{display:block;width:100%;height:100%;object-fit:contain}.empty{position:absolute;text-align:center;padding:20px;color:#a2b7aa}
    .tag{position:absolute;left:10px;top:10px;background:#102019cc;border:1px solid #45614f;border-radius:4px;padding:2px 7px;font-size:11px;color:#b9e0c7}
    .tools{display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin-top:10px;font-size:12px}
    .info{align-self:center}.info strong{font-size:16px}.info p{color:#a8bcb0;font-size:12px;max-width:480px;margin:10px 0}
    .foot{border-top:1px solid #2a4032;padding:9px 16px;display:flex;justify-content:space-between;gap:8px;flex-wrap:wrap;font-size:12px;color:#a8bcb0}
    .status{overflow-wrap:anywhere}.metrics{color:#75dca6;font-variant-numeric:tabular-nums}a{color:#77dca9}
    :host([data-floating]){position:fixed;width:390px;max-width:calc(100vw - 24px);right:16px;bottom:16px;z-index:1000;margin:0;max-height:calc(100dvh - 32px);overflow:auto;box-shadow:0 12px 60px #0009;border-radius:10px}
    :host([data-floating]) .body{display:block;padding:0 12px 12px}:host([data-floating]) .info{display:none}
    :host([data-floating]) .head{padding:12px}:host([data-floating]) .settings{width:100%}:host([data-floating]) input{flex:1;min-width:120px;width:160px}
    .view:fullscreen{width:100vw;height:100vh;aspect-ratio:auto;border:0;border-radius:0}
    @media(max-width:680px){.head{padding:12px}.settings{width:100%;order:3}.settings label{flex:1}input{width:100%}.body{display:block;padding:0 12px 12px}.info{display:none}.actions{gap:5px}button{padding:7px 8px}.foot{padding:9px 12px}:host([data-floating]){right:12px;bottom:12px}}
    </style>
    <section class="card" aria-label="车载摄像头">
      <div class="head">
        <strong class="title"><i id="dot" class="dot"></i>车载画面</strong>
        <div class="settings">
          <label>摄像头地址 <input id="address" type="text" inputmode="url" autocomplete="off" spellcheck="false" aria-label="摄像头地址" placeholder="填串口显示的摄像头 IP"></label>
          <label>帧率 <select id="rate" aria-label="视频帧率"><option value="10">10 帧/秒</option><option value="5">5 帧/秒</option><option value="3">3 帧/秒</option></select></label>
        </div>
        <div class="actions"><button id="toggle" class="primary" type="button">开启画面</button><button id="float" type="button" title="悬浮后可同时查看下方路线">悬浮</button></div>
      </div>
      <div id="body" class="body" hidden>
        <div><div id="view" class="view"><img id="frame" alt="车载摄像头实时画面" hidden><span id="empty" class="empty">等待摄像头</span><span id="tag" class="tag">尚无新帧</span></div>
          <div class="tools"><button id="full" type="button">放大画面</button><a id="direct" target="_blank" rel="noopener noreferrer">单独打开摄像头</a></div></div>
        <div class="info"><strong>前方视野，与扫描同步查看</strong>
          <p>仅在一个窗口开启视频。若手控变卡，先关闭画面对比，再降至 5 或 3 帧/秒。</p>
          <p>这是低帧率辅助图传，不保证无延迟。画面中断或过期时，请立即停止驾驶。</p>
          <p>“悬浮”可让画面跟随页面滚动；“单独打开”会先关闭本窗口收图，等 3 秒后可在新窗口开启。</p>
        </div>
      </div>
      <div class="foot"><span id="status" class="status" role="status">默认关闭 · 摄像头与电脑须连接同一个基座热点</span><span id="metrics" class="metrics">320 × 240 · 图传接口 1.0</span></div>
    </section>`;
  const header = main.querySelector('header');
  if (header) header.after(host); else main.prepend(host);
  const $ = id => ui.getElementById(id);
  const STORAGE = 'gimbal-camera-v1';
  try {
    const saved = JSON.parse(localStorage.getItem(STORAGE) || '{}');
    $('address').value = typeof saved.address === 'string' ? saved.address : 'http://rover-camera.local';
    $('rate').value = ['3', '5', '10'].includes(saved.rate) ? saved.rate : '10';
  } catch (_) { $('address').value = 'http://rover-camera.local'; }

  let active = false, generation = 0, controller = null, timer = 0;
  let currentUrl = '', lastFrameId = '', lastFrameAt = 0, previousFrameAt = 0, failures = 0;
  function address() {
    let text = $('address').value.trim();
    if (!text) throw new Error('先填写摄像头串口显示的 IP 地址。');
    if (!/^https?:\/\//i.test(text)) text = 'http://' + text;
    const url = new URL(text);
    const privateIp = /^(10\.|192\.168\.|172\.(1[6-9]|2\d|3[01])\.)/.test(url.hostname) &&
      url.hostname.split('.').length === 4 && url.hostname.split('.').every(n => /^\d{1,3}$/.test(n) && Number(n) <= 255);
    if (url.protocol !== 'http:' || url.username || url.password ||
        !(privateIp || /^[a-z0-9-]+\.local$/i.test(url.hostname))) {
      throw new Error('请填写 http:// 开头的局域网摄像头 IP，或 rover-camera.local。');
    }
    if (url.hostname === '192.168.4.1') throw new Error('192.168.4.1 是基座，不是摄像头；请看摄像头串口的 Camera URL。');
    if (!['/', '/capture'].includes(url.pathname)) throw new Error('只填摄像头地址，例如 http://192.168.4.3，不填其他路径。');
    return url.origin;
  }
  function message(text, state = '') {
    $('status').textContent = text;
    $('dot').dataset.state = state;
  }
  function clearFrame(text) {
    $('frame').hidden = true;
    $('frame').removeAttribute('src');
    if (currentUrl) URL.revokeObjectURL(currentUrl);
    currentUrl = '';
    $('empty').hidden = false;
    $('empty').textContent = text;
    $('tag').textContent = '尚无新帧';
  }
  function stop(text = '画面已关闭 · 不影响原有遥控与扫描', error = false) {
    active = false;
    generation++;
    clearTimeout(timer);
    if (controller) controller.abort();
    controller = null;
    lastFrameAt = 0;
    clearFrame(error ? '画面不可用，请停止驾驶' : '画面已关闭');
    $('body').hidden = !error;
    $('toggle').textContent = '开启画面';
    $('address').disabled = false;
    $('rate').disabled = false;
    $('metrics').textContent = '320 × 240 · 图传接口 1.0';
    message(text, error ? 'error' : '');
  }
  async function poll(g, origin, viewer) {
    if (!active || generation !== g) return;
    const began = performance.now(), abort = new AbortController();
    controller = abort;
    const deadline = setTimeout(() => abort.abort(), 1200);
    let pendingUrl = '', nextDelay = 1000;
    try {
      const response = await fetch(origin + '/capture?viewer=' + viewer + '&t=' + Date.now(),
        {mode:'cors', cache:'no-store', credentials:'omit', signal:abort.signal});
      if (response.status === 409) {
        stop('已有其他窗口收图。先关闭另一页的视频，等 3 秒再开启。', true);
        return;
      }
      if (response.status === 429) { nextDelay = 250; return; }
      if (!response.ok) throw new Error('摄像头响应 ' + response.status + '；可打开摄像头 /status 检查');
      if (response.headers.get('X-Camera-Version') !== '1.0')
        throw new Error('摄像头接口不匹配，请烧录 ESP32CAM_Camera V1.2 固件');
      const frameId = response.headers.get('X-Frame-Id'), ageText = response.headers.get('X-Frame-Age-Ms');
      const age = ageText === null ? NaN : Number(ageText);
      if (!frameId || frameId === lastFrameId || !Number.isFinite(age) || age < 0 || age > 500)
        throw new Error('没有收到新鲜画面');
      const blob = await response.blob();
      if (blob.type !== 'image/jpeg' || blob.size < 100 || blob.size > 160000)
        throw new Error('图像数据不完整或格式不符');
      if (generation !== g) return;
      pendingUrl = URL.createObjectURL(blob);
      const decoded = new Image();
      decoded.src = pendingUrl;
      await decoded.decode();
      if (generation !== g) return;
      if (abort.signal.aborted || performance.now() - began > 1000) throw new Error('收图耗时超过 1 秒');
      if (currentUrl) URL.revokeObjectURL(currentUrl);
      currentUrl = pendingUrl;
      pendingUrl = '';
      $('frame').src = currentUrl;
      $('frame').hidden = false;
      $('empty').hidden = true;
      lastFrameId = frameId;
      lastFrameAt = performance.now();
      const fps = previousFrameAt ? Math.min(99, 1000 / (lastFrameAt - previousFrameAt)).toFixed(1) : '--';
      previousFrameAt = lastFrameAt;
      failures = 0;
      $('tag').textContent = '最新画面';
      $('metrics').textContent = fps + ' 帧/秒 · 收图 ' + Math.round(lastFrameAt - began) + ' ms';
      message('持续收图 · 此耗时不是端到端延迟', 'live');
      // 10帧档至少105ms，留5ms调度余量；等待本次完成，不并发、不补发积压帧。
      nextDelay = Math.max(105, 1000 / Number($('rate').value)) - (performance.now() - began);
    } catch (error) {
      if (generation !== g) return;
      failures++;
      clearFrame('画面中断，请停止驾驶');
      $('metrics').textContent = '无可用实时画面';
      if (failures >= 3) {
        stop('连接失败：' + (error.name === 'AbortError' ? '收图超时' : error.message) +
          '。核对 IP、同一热点和浏览器的局域网访问权限；也可先单独打开摄像头。', true);
      } else message('画面中断，请停止驾驶 · 正在重试 ' + failures + '/3', 'error');
    } finally {
      clearTimeout(deadline);
      if (pendingUrl) URL.revokeObjectURL(pendingUrl);
      if (controller === abort) controller = null;
      if (active && generation === g) timer = setTimeout(() => poll(g, origin, viewer), Math.max(0, nextDelay));
    }
  }
  function start() {
    let origin;
    try { origin = address(); }
    catch (error) { message(error.message, 'error'); return; }
    stop();
    active = true;
    lastFrameId = '';
    previousFrameAt = 0;
    failures = 0;
    $('address').value = origin;
    $('address').disabled = true;
    $('rate').disabled = true;
    $('toggle').textContent = '关闭画面';
    $('direct').href = origin + '/';
    $('body').hidden = false;
    clearFrame('正在连接摄像头…');
    message('正在连接摄像头，与基座遥测连接相互独立');
    try { localStorage.setItem(STORAGE, JSON.stringify({address:origin,rate:$('rate').value})); } catch (_) {}
    const viewer = Array.from(crypto.getRandomValues(new Uint8Array(16)), n => n.toString(16).padStart(2,'0')).join('');
    poll(generation, origin, viewer);
  }
  $('toggle').addEventListener('click', () => active ? stop() : start());
  $('float').addEventListener('click', () => {
    host.toggleAttribute('data-floating');
    $('float').textContent = host.hasAttribute('data-floating') ? '嵌回页面' : '悬浮';
  });
  $('direct').addEventListener('click', () => stop('本窗口已关闭，单独页面请等 3 秒再开启画面。'));
  $('full').addEventListener('click', async () => {
    try {
      if (document.fullscreenElement) await document.exitFullscreen();
      else if ($('view').requestFullscreen) await $('view').requestFullscreen();
      else message('此浏览器不支持全屏；可使用“单独打开摄像头”。');
    } catch (_) { message('浏览器未允许全屏；仍可正常收图。'); }
  });
  setInterval(() => {
    if (!active || !lastFrameAt) return;
    const age = performance.now() - lastFrameAt;
    if (age > 1000) {
      clearFrame('画面过期，请停止驾驶');
      $('metrics').textContent = '超过 1 秒没有新画面';
      message('画面过期，不要依赖旧画面驾驶', 'error');
    }
  }, 200);
  document.addEventListener('visibilitychange', () => {
    if (document.hidden && active) stop('页面已隐藏，视频自动关闭；返回后请重新开启。');
  });
  window.addEventListener('pagehide', () => stop());
})();
