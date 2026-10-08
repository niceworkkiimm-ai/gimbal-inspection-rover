(function () {
  "use strict";

  const MAX_DISTANCE = 400;
  const POINT_LIFETIME = 6500;
  const canvas = document.getElementById("radarCanvas");
  const context = canvas.getContext("2d");

  const elements = {
    connectionState: document.getElementById("connectionState"),
    connectionText: document.getElementById("connectionText"),
    demoMode: document.getElementById("demoMode"),
    deviceMode: document.getElementById("deviceMode"),
    pauseButton: document.getElementById("pauseButton"),
    clearButton: document.getElementById("clearButton"),
    distanceValue: document.getElementById("distanceValue"),
    distanceBar: document.getElementById("distanceBar"),
    objectState: document.getElementById("objectState"),
    yawValue: document.getElementById("yawValue"),
    pitchValue: document.getElementById("pitchValue"),
    sequenceValue: document.getElementById("sequenceValue"),
    rateValue: document.getElementById("rateValue"),
    sampleTime: document.getElementById("sampleTime"),
    targetsList: document.getElementById("targetsList"),
    targetCount: document.getElementById("targetCount"),
    sourceLabel: document.getElementById("sourceLabel"),
    packetState: document.getElementById("packetState")
    ,gasGrid: document.getElementById("gasGrid")
  };

  const state = {
    mode: "demo",
    paused: false,
    sequence: 0,
    yaw: -72,
    pitch: 0,
    distance: null,
    valid: false,
    direction: 1,
    points: [],
    lastSampleAt: 0,
    recentSamples: [],
    deviceTimer: null,
    deviceRequest: null,
    deviceGeneration: 0,
    deviceSequence: null,
    gases: []
  };

  function setConnection(type, label) {
    elements.connectionState.dataset.state = type;
    elements.connectionText.textContent = label;
  }

  function setMode(mode) {
    state.mode = mode;
    state.gases = [];
    state.points = [];
    state.recentSamples = [];
    state.valid = false;
    state.distance = null;
    updateTelemetry();
    elements.demoMode.classList.toggle("is-active", mode === "demo");
    elements.deviceMode.classList.toggle("is-active", mode === "device");
    elements.demoMode.setAttribute("aria-pressed", String(mode === "demo"));
    elements.deviceMode.setAttribute("aria-pressed", String(mode === "device"));
    elements.sourceLabel.textContent = mode === "demo" ? "模拟器" : "ESP-01S";

    if (mode === "demo") {
      disconnectDevice();
      setConnection("demo", "演示数据");
      elements.packetState.textContent = "正常";
    } else {
      connectDevice();
    }
  }

  function connectDevice() {
    disconnectDevice();
    const generation = state.deviceGeneration;
    state.deviceSequence = null;
    setConnection("demo", "正在连接设备");
    elements.packetState.textContent = "连接中";
    // Same device API as the standalone pages, including localhost/file hosting.
    const base = "http://192.168.4.1";
    const poll = async function () {
      if (state.deviceRequest || state.mode !== "device" || generation !== state.deviceGeneration) return;
      const controller = new AbortController();
      state.deviceRequest = controller;
      const timeout = setTimeout(() => controller.abort(), 1200);
      try {
        const response = await fetch(`${base}/api/telemetry?t=${Date.now()}`,
          {cache:"no-store", signal:controller.signal});
        if (!response.ok) throw new Error("HTTP");
        const frame = await response.json();
        if (state.mode !== "device" || generation !== state.deviceGeneration) return;
        state.gases = Array.isArray(frame.gases) ? frame.gases : [];
        updateGas(state.gases); // Independent of pause, duplicate scan or T-frame loss.
        if (frame.online !== true) {
          setConnection("offline", "等待主控");
          elements.packetState.textContent = "中断";
          state.valid = false;
          state.distance = null;
          updateTelemetry();
          return;
        }
        setConnection("online", frame.link ? "设备在线" : "手持端失联");
        elements.packetState.textContent = frame.link ? "正常" : "姿态失联";
        if (frame.sequence === state.deviceSequence) return;
        state.deviceSequence = frame.sequence;
        addReading({sequence: Number(frame.sequence), yaw:-Number(frame.roll),
          pitch:-Number(frame.pitch), distance:Number(frame.distance),
          valid:frame.distanceValid === true && frame.link === true});
      } catch (error) {
        if (state.mode !== "device" || generation !== state.deviceGeneration) return;
        setConnection("offline", "设备未连接");
        elements.packetState.textContent = "中断";
        state.gases = [];
        state.valid = false;
        state.distance = null;
        state.deviceSequence = null;
        updateTelemetry();
      } finally {
        clearTimeout(timeout);
        if (state.deviceRequest === controller) state.deviceRequest = null;
      }
    };
    state.deviceTimer = setInterval(poll, 250);
    poll();
  }

  function disconnectDevice() {
    state.deviceGeneration++;
    clearInterval(state.deviceTimer);
    state.deviceTimer = null;
    if (state.deviceRequest) state.deviceRequest.abort();
    state.deviceRequest = null;
  }

  function simulateReading() {
    if (state.mode !== "demo" || state.paused) return;

    state.yaw += state.direction * 1.8;
    if (state.yaw >= 74) {
      state.yaw = 74;
      state.direction = -1;
    } else if (state.yaw <= -74) {
      state.yaw = -74;
      state.direction = 1;
    }

    state.pitch = 7.5 * Math.sin((state.yaw + 20) * Math.PI / 90);
    const objects = [
      { angle: -47, width: 8, distance: 126 },
      { angle: 10, width: 11, distance: 72 },
      { angle: 52, width: 7, distance: 184 }
    ];
    const hit = objects.find(function (object) {
      return Math.abs(state.yaw - object.angle) <= object.width;
    });

    const reading = {
      sequence: ++state.sequence,
      yaw: state.yaw,
      pitch: state.pitch,
      distance: hit ? hit.distance + Math.sin(state.sequence * 0.7) * 3.4 : 0,
      valid: Boolean(hit)
    };
    addReading(reading);
  }

  function addReading(reading) {
    if (state.paused) return;
    const now = performance.now();
    const validNumbers = Number.isFinite(reading.yaw) &&
      Number.isFinite(reading.pitch) && Number.isFinite(reading.distance);
    if (!validNumbers) return;

    state.sequence = reading.sequence;
    state.yaw = Math.max(-90, Math.min(90, reading.yaw));
    state.pitch = Math.max(-90, Math.min(90, reading.pitch));
    state.valid = reading.valid && reading.distance >= 2 && reading.distance <= MAX_DISTANCE;
    state.distance = state.valid ? reading.distance : null;
    state.lastSampleAt = now;
    state.recentSamples.push(now);
    state.recentSamples = state.recentSamples.filter(function (time) {
      return now - time <= 1000;
    });

    if (state.valid) {
      state.points.push({
        yaw: state.yaw,
        pitch: state.pitch,
        distance: state.distance,
        createdAt: now
      });
    }

    state.points = state.points.filter(function (point) {
      return now - point.createdAt <= POINT_LIFETIME;
    });
    updateTelemetry();
  }

  function updateTelemetry() {
    elements.yawValue.textContent = state.yaw.toFixed(1);
    elements.pitchValue.textContent = state.pitch.toFixed(1);
    elements.sequenceValue.textContent = String(state.sequence);
    elements.rateValue.textContent = String(state.recentSamples.length);
    elements.sampleTime.textContent = `最近数据 ${new Date().toLocaleTimeString("zh-CN", { hour12: false })}`;

    if (state.valid) {
      const rounded = state.distance.toFixed(1);
      elements.distanceValue.textContent = rounded;
      elements.distanceBar.style.width = `${Math.max(2, 100 - state.distance / MAX_DISTANCE * 100)}%`;
      elements.distanceBar.style.backgroundColor = state.distance < 20 ? "var(--red)" :
        state.distance < 100 ? "var(--amber)" : "var(--green)";
      elements.objectState.textContent = state.distance < 20 ? "距离过近" : "检测到目标";
      elements.objectState.dataset.state = state.distance < 20 ? "danger" : "detected";
    } else {
      elements.distanceValue.textContent = "---";
      elements.distanceBar.style.width = "0";
      elements.objectState.textContent = "无目标";
      elements.objectState.dataset.state = "clear";
    }

    updateTargets();
    updateGas(state.gases);
  }

function updateGas(gases) {
  const grid = document.getElementById('gasGrid');
  if (!grid) return;
  const rows = Array.isArray(gases) && gases.length ? gases :
    [1,2,3,4].map(channel => ({channel, name:'待识别气体'}));
  grid.replaceChildren();
  rows.forEach(g => {
    const online = g.online === true;
    const schemaOk = typeof g.unit === 'string' && typeof g.statusText === 'string';
    const valid = online && schemaOk && g.valid === true &&
      typeof g.concentration === 'number' && Number.isFinite(g.concentration);
    const allowed = ['normal','warning','alarm','fault','warmup','offline'];
    let level = online && schemaOk && allowed.includes(g.severity) ? g.severity : 'offline';
    if (online && (!schemaOk || (level === 'normal' && !valid))) level = 'fault';
    const card = document.createElement('div');
    card.className = 'gas-item';
    card.dataset.level = level;
    const value = document.createElement('strong');
    const decimals = Number.isInteger(g.decimals) && g.decimals >= 0 && g.decimals <= 3 ? g.decimals : 0;
    value.textContent = (g.name || '待识别气体') + ': ' +
      (valid ? g.concentration.toFixed(decimals) + ' ' + g.unit : '--');
    const detail = document.createElement('small');
    detail.textContent = (online ? schemaOk ? g.statusText : '需更新基座ESP' : '未连接 / 无新数据') +
      ' · 通道 ' + (g.channel || '?');
    card.append(value, detail);
    grid.append(card);
  });
}

  function clusterTargets() {
    const now = performance.now();
    const recent = state.points
      .filter(function (point) { return now - point.createdAt < 4800; })
      .slice()
      .sort(function (a, b) { return a.yaw - b.yaw; });
    const clusters = [];

    recent.forEach(function (point) {
      const last = clusters[clusters.length - 1];
      if (last && Math.abs(point.yaw - last.yaw) < 16 &&
          Math.abs(point.distance - last.distance) < 24) {
        last.count += 1;
        last.yaw += (point.yaw - last.yaw) / last.count;
        last.pitch += (point.pitch - last.pitch) / last.count;
        last.distance += (point.distance - last.distance) / last.count;
        last.createdAt = Math.max(last.createdAt, point.createdAt);
      } else {
        clusters.push({
          count: 1,
          yaw: point.yaw,
          pitch: point.pitch,
          distance: point.distance,
          createdAt: point.createdAt
        });
      }
    });

    return clusters
      .filter(function (cluster) { return cluster.count >= 2; })
      .sort(function (a, b) { return a.distance - b.distance; })
      .slice(0, 4);
  }

  function directionLabel(angle) {
    if (Math.abs(angle) < 5) return "正前方";
    return angle < 0 ? `左侧 ${Math.abs(angle).toFixed(0)}°` : `右侧 ${angle.toFixed(0)}°`;
  }

  function updateTargets() {
    const targets = clusterTargets();
    elements.targetCount.textContent = String(targets.length);

    if (!targets.length) {
      elements.targetsList.innerHTML = '<p class="empty-state">扫描区域暂无有效目标</p>';
      return;
    }

    elements.targetsList.innerHTML = targets.map(function (target, index) {
      const nearClass = target.distance < 20 ? " is-near" : "";
      return `<div class="target-row${nearClass}">
        <span class="target-index">${index + 1}</span>
        <span class="target-direction"><strong>${directionLabel(target.yaw)}</strong><small>俯仰 ${target.pitch.toFixed(1)}°</small></span>
        <span class="target-distance">${target.distance.toFixed(1)} cm</span>
      </div>`;
    }).join("");
  }

  function resizeCanvas() {
    const bounds = canvas.getBoundingClientRect();
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.round(bounds.width * ratio);
    canvas.height = Math.round(bounds.height * ratio);
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
  }

  function radarPoint(yaw, distance, geometry) {
    const angle = (yaw - 90) * Math.PI / 180;
    const radius = Math.max(0, Math.min(1, distance / MAX_DISTANCE)) * geometry.radius;
    return {
      x: geometry.cx + Math.cos(angle) * radius,
      y: geometry.cy + Math.sin(angle) * radius
    };
  }

  function drawRadar() {
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    if (!width || !height) {
      requestAnimationFrame(drawRadar);
      return;
    }

    const styles = getComputedStyle(document.documentElement);
    const geometry = {
      cx: width / 2,
      cy: height - 24,
      radius: Math.min(width * 0.47, height - 42)
    };
    const now = performance.now();

    context.clearRect(0, 0, width, height);
    context.fillStyle = styles.getPropertyValue("--radar");
    context.fillRect(0, 0, width, height);

    context.save();
    context.strokeStyle = styles.getPropertyValue("--radar-grid");
    context.fillStyle = "#779381";
    context.lineWidth = 1;
    context.font = '11px "Microsoft YaHei UI", sans-serif';
    context.textAlign = "center";

    [0.25, 0.5, 0.75, 1].forEach(function (fraction) {
      context.beginPath();
      context.arc(geometry.cx, geometry.cy, geometry.radius * fraction, Math.PI, Math.PI * 2);
      context.stroke();
      if (fraction < 1) {
        context.fillText(`${Math.round(MAX_DISTANCE * fraction)}`, geometry.cx + 4, geometry.cy - geometry.radius * fraction + 14);
      }
    });

    [-75, -45, -15, 15, 45, 75].forEach(function (angle) {
      const end = radarPoint(angle, MAX_DISTANCE, geometry);
      context.beginPath();
      context.moveTo(geometry.cx, geometry.cy);
      context.lineTo(end.x, end.y);
      context.stroke();
      context.fillText(`${angle > 0 ? "+" : ""}${angle}°`, end.x, Math.max(15, end.y - 8));
    });
    context.restore();

    state.points.forEach(function (point) {
      const age = now - point.createdAt;
      const alpha = Math.max(0.12, 1 - age / POINT_LIFETIME);
      const position = radarPoint(point.yaw, point.distance * Math.cos(point.pitch * Math.PI / 180), geometry);
      const color = point.distance < 20 ? "211, 74, 63" :
        point.distance < 100 ? "228, 165, 28" : "24, 167, 102";

      context.beginPath();
      context.fillStyle = `rgba(${color}, ${alpha * 0.2})`;
      context.arc(position.x, position.y, 9, 0, Math.PI * 2);
      context.fill();
      context.beginPath();
      context.fillStyle = `rgba(${color}, ${alpha})`;
      context.arc(position.x, position.y, 3.2, 0, Math.PI * 2);
      context.fill();
    });

    const sweepEnd = radarPoint(state.yaw, MAX_DISTANCE, geometry);
    context.save();
    context.strokeStyle = styles.getPropertyValue("--amber");
    context.lineWidth = 2;
    context.beginPath();
    context.moveTo(geometry.cx, geometry.cy);
    context.lineTo(sweepEnd.x, sweepEnd.y);
    context.stroke();
    context.fillStyle = styles.getPropertyValue("--amber");
    context.beginPath();
    context.arc(geometry.cx, geometry.cy, 5, 0, Math.PI * 2);
    context.fill();
    context.restore();

    requestAnimationFrame(drawRadar);
  }

  elements.demoMode.addEventListener("click", function () { setMode("demo"); });
  elements.deviceMode.addEventListener("click", function () { setMode("device"); });
  elements.pauseButton.addEventListener("click", function () {
    state.paused = !state.paused;
    elements.pauseButton.textContent = state.paused ? "继续" : "暂停";
  });
  elements.clearButton.addEventListener("click", function () {
    state.points = [];
    updateTargets();
  });

  const observer = new ResizeObserver(resizeCanvas);
  observer.observe(canvas);
  resizeCanvas();
  updateTelemetry();
  setInterval(simulateReading, 55);
  requestAnimationFrame(drawRadar);
})();
