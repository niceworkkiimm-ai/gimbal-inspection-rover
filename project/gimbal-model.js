/* Video rover geometry shared by live scans and saved 3D snapshots. */
(function(root) {
  'use strict';

  const SCENE_UNITS_PER_METER = 400;
  const WHEEL_MAX_AGE_MS = 1000;
  const RPM10_TO_RADIANS_PER_SECOND = Math.PI * 2 / 600;
  const WHEEL_NAMES = ['wheel-fl', 'wheel-fr', 'wheel-rl', 'wheel-rr'];
  const WHEEL_GEOMETRY = ['leftHelix', 'rightHelix', 'rightHelix', 'leftHelix'];

  function bytes(base64) {
    return Uint8Array.from(root.atob(base64), character => character.charCodeAt(0)).buffer;
  }

  function create(THREE) {
    const data = root.RoverModelData;
    if (!data) throw Error('缺少新版小车模型，请保留同目录的 rover-model-data.js。');
    const materials = data.materials.map(([r, g, b, roughness, metalness]) =>
      new THREE.MeshStandardMaterial({
        color: new THREE.Color().setRGB(r, g, b),
        roughness,
        metalness: Math.min(metalness, .5)
      }));
    const vector = values => new THREE.Vector3(...values).multiplyScalar(SCENE_UNITS_PER_METER);
    const geometry = {};
    for (const [name, batches] of Object.entries(data.geometry)) {
      geometry[name] = batches.map(([material, positions, normals, indices]) => {
        const shape = new THREE.BufferGeometry();
        const quantized = new Int16Array(bytes(positions));
        const vertices = Float32Array.from(quantized, value => value * SCENE_UNITS_PER_METER / 40000);
        shape.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
        shape.setAttribute('normal', new THREE.BufferAttribute(new Int8Array(bytes(normals)), 3, true));
        shape.setIndex(new THREE.BufferAttribute(new Uint16Array(bytes(indices)), 1));
        return {shape, material:materials[material]};
      });
    }

    function addPart(parent, name, batches, offset) {
      const part = new THREE.Group();
      part.name = name;
      if (offset) part.position.copy(vector(offset));
      for (const {shape, material} of geometry[batches]) part.add(new THREE.Mesh(shape, material));
      parent.add(part);
      return part;
    }

    const machine = new THREE.Group();
    machine.name = 'inspection-rover';
    addPart(machine, 'purple-cad-body', 'body');
    const yawAssembly = addPart(machine, 'gimbal-yaw', 'pan', data.pan);
    const pitchAssembly = addPart(yawAssembly, 'gimbal-pitch', 'pitch', data.pitch);
    const wheelPivots = data.wheels.map((offset, i) =>
      addPart(machine, WHEEL_NAMES[i], WHEEL_GEOMETRY[i], offset));
    const sensor = vector(data.sensor);
    const pitchAxis = new THREE.Vector3(1, 0, 0);
    const yawAxis = new THREE.Vector3(0, 1, 0);
    function pointAt(distance, yawDegrees, pitchDegrees) {
      return new THREE.Vector3(sensor.x, sensor.y, sensor.z + distance)
        .applyAxisAngle(pitchAxis, -THREE.MathUtils.degToRad(pitchDegrees))
        .add(pitchAssembly.position)
        .applyAxisAngle(yawAxis, THREE.MathUtils.degToRad(yawDegrees))
        .add(yawAssembly.position);
    }

    let wheelVelocity = [0, 0, 0, 0], expiresAt = 0, lastStep = null, sequence = null;
    function advanceWheels(now) {
      if (lastStep !== null && now <= lastStep) return;
      const seconds = lastStep === null ? 0 : Math.max(0, Math.min(now, expiresAt) - lastStep) / 1000;
      lastStep = now;
      if (!seconds) return;
      wheelPivots.forEach((wheel, i) => {
        wheel.rotation.x = (wheel.rotation.x + wheelVelocity[i] * seconds) % (2 * Math.PI);
      });
    }

    function setWheelSpeed(sample, now, rate = 1) {
      advanceWheels(now);
      if (!sample || sample.online !== true || sample.valid !== true ||
          !Number.isFinite(sample.ageMs) || sample.ageMs < 0 || sample.ageMs >= WHEEL_MAX_AGE_MS ||
          !Array.isArray(sample.measuredRpm10) || sample.measuredRpm10.length !== 4 ||
          !sample.measuredRpm10.every(Number.isFinite)) {
        expiresAt = now;
        sequence = null;
        return;
      }
      const deadline = now + WHEEL_MAX_AGE_MS - sample.ageMs;
      // Polling the same sample must not extend its freshness.
      expiresAt = sequence === sample.sequence ? Math.min(expiresAt, deadline) : deadline;
      sequence = sample.sequence;
      // STM32 already applies the encoder direction calibration. Positive RPM
      // rolls toward the camera end on all four pivots, with +X as the axle.
      wheelVelocity = sample.measuredRpm10.map(value => value * RPM10_TO_RADIANS_PER_SECOND * rate);
    }

    return {machine, yawAssembly, pitchAssembly, pointAt, wheelPivots, advanceWheels, setWheelSpeed};
  }

  const api = {create};
  if (typeof module === 'object' && module.exports) module.exports = api;
  root.GimbalModel = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
