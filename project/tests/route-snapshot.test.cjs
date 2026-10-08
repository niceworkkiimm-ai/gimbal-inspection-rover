'use strict';
// Run: node tests/route-snapshot.test.cjs. No browser, network or hardware access.
const assert = require('node:assert/strict');
const { RouteEngine, defaults, captureScan, routeRecord, parseRouteFile } = require('../vehicle-route.js');
const now = Date.now() + 1000;
const copy = value => JSON.parse(JSON.stringify(value));
let passed = 0, failed = 0;
function check(name, test) {
  try { test(); ++passed; console.log(`PASS ${name}`); }
  catch (error) { ++failed; console.error(`FAIL ${name}\n${error.stack}`); }
}
function liveRoute() {
  const route = new RouteEngine({ ...defaults(), signs: [1, 1, 1, 1], confirmed: true });
  route.originTime = now - 1000;
  route.feed({ protocol: 1, session: 'test-session', online: true, samples: [[1, 0, 0, 0, 0, 0]] }, now - 80);
  route.feed({ protocol: 1, session: 'test-session', online: true, samples: [[2, 80, 10, 10, 10, 10]] }, now);
  return route;
}
function scan(kind = '2d') {
  const value = { kind, points: [{ distance: 25, yaw: -30, pitch: 45 }, { distance: 70, yaw: 60, pitch: 90 }] };
  if (kind === '3d') value.view = { camera: [1, 2, 3], target: [0, 0, 0] };
  return value;
}
function withScan() {
  const route = liveRoute();
  captureScan(route, scan(), now);
  return route;
}
function invalidWithoutMutation(route, action) {
  const before = copy(route);
  assert.throws(action);
  assert.deepEqual(copy(route), before);
}

check('2D/3D snapshots survive version 2 save/open with their route poses', () => {
  const route = liveRoute();
  const first = captureScan(route, scan(), now);
  const second = captureScan(route, scan('3d'), now + 1);
  assert.ok(Number.isInteger(first.id) && first.id > 0 && second.id !== first.id);
  assert.deepEqual(first.pose, { x: route.x, y: route.y, heading: route.heading, time: route.elapsed, distance: route.distance });
  assert.equal(first.capturedAt, now);
  const record = routeRecord(route, now + 2);
  assert.equal(record.format, 'gimbal-route');
  assert.equal(record.version, 2);
  assert.deepEqual(parseRouteFile(JSON.stringify(record)), record.route);
  assert.deepEqual(record.route.scans, [first, second]);
});

check('capture and export own their data and do not change route points or pose', () => {
  const route = liveRoute(), source = scan('3d'), before = copy(route);
  const checkpoint = captureScan(route, source, now);
  source.points[0].distance = 149;
  source.view.camera[0] = 999;
  assert.equal(checkpoint.scan.points[0].distance, 25);
  assert.equal(checkpoint.scan.view.camera[0], 1);
  const after = copy(route);
  delete before.scans; delete after.scans;
  assert.deepEqual(after, before);
  const record = routeRecord(route, now);
  record.route.scans[0].scan.points[0].distance = 148;
  record.route.scans[0].pose.x = 100;
  record.route.config.signs[0] = -1;
  record.route.points[0].x = 100;
  assert.equal(route.scans[0].scan.points[0].distance, 25);
  assert.equal(route.scans[0].pose.x, route.x);
  assert.equal(route.config.signs[0], 1);
  assert.deepEqual(route.points, before.points);
});

check('legacy route v1 and inspection v1 load only the old route', () => {
  const route = routeRecord(liveRoute(), now).route;
  delete route.scans;
  for (const format of ['gimbal-route', 'gimbal-inspection']) {
    const opened = parseRouteFile('\uFEFF' + JSON.stringify({ format, version: 1, route, stations: [{ maliciousOrObsolete: true }] }));
    assert.deepEqual(opened.points, route.points);
    assert.deepEqual(opened.scans || [], []);
    assert.equal(opened.stations, undefined);
  }
});

check('bad imports reject unknown format/version, malformed data and non-finite numbers', () => {
  const live = withScan(), before = copy(live), good = routeRecord(live, now);
  for (const mutate of [
    record => { record.format = 'unknown'; },
    record => { record.version = 99; },
    record => { record.route.scans[0].id = 0; },
    record => { record.route.scans.push(copy(record.route.scans[0])); },
    record => { record.route.scans[0].scan.kind = 'image'; },
    record => { record.route.scans[0].scan.points[0].distance = 151; },
    record => { record.route.scans[0].scan.points[0].yaw = -181; },
    record => { record.route.scans[0].scan.points[0].pitch = 181; },
    record => { record.route.scans[0].pose.heading = null; },
    record => { record.route.scans[0].scan.view = { camera: [0, 1], target: [0, 0, 0] }; },
    record => { record.route.scans[0].scan.points = Array.from({ length: 801 }, () => ({ distance: 20, yaw: 0, pitch: 0 })); },
    record => { record.route.scans = Array.from({ length: 101 }, (_, i) => ({ ...copy(record.route.scans[0]), id: i + 1 })); }
  ]) {
    const bad = copy(good); mutate(bad);
    assert.throws(() => parseRouteFile(JSON.stringify(bad)));
  }
  assert.throws(() => parseRouteFile('{broken json'));
  assert.throws(() => parseRouteFile(JSON.stringify(good).replace('"distance":25', '"distance":1e400')));
  const nonFinite = withScan(); nonFinite.scans[0].scan.points[0].yaw = NaN;
  assert.throws(() => routeRecord(nonFinite, now));
  assert.deepEqual(copy(live), before);
});

check('offline, stale, uncalibrated and unsynchronized routes cannot gain a checkpoint', () => {
  for (const prepare of [
    route => { route.online = false; },
    route => { route.lastReceived = now - 1201; },
    route => { route.config.confirmed = false; },
    route => { route.nativeLog = true; route.logPending = true; }
  ]) {
    const route = liveRoute(); prepare(route);
    invalidWithoutMutation(route, () => captureScan(route, scan(), now));
  }
  const synced = liveRoute(); synced.nativeLog = true; synced.logPending = false;
  captureScan(synced, scan(), now);
  assert.equal(synced.scans.length, 1);
});

check('snapshot limits reject excess without losing already captured scans', () => {
  const route = liveRoute();
  const full = scan(); full.points = Array.from({ length: 800 }, () => ({ distance: 150, yaw: -180, pitch: 180 }));
  captureScan(route, full, now);
  const tooMany = scan(); tooMany.points = Array.from({ length: 801 }, () => ({ distance: 20, yaw: 0, pitch: 0 }));
  invalidWithoutMutation(route, () => captureScan(route, tooMany, now));
  for (let i = 1; i < 100; ++i) captureScan(route, scan(), now);
  assert.equal(route.scans.length, 100);
  assert.equal(new Set(route.scans.map(checkpoint => checkpoint.id)).size, 100);
  invalidWithoutMutation(route, () => captureScan(route, scan(), now));
});

check('restart and calibration changes clear scans with the route', () => {
  const route = withScan(); route.reset();
  assert.deepEqual(route.scans, []);
  assert.deepEqual(route.points, []);
  const configured = withScan(); configured.configure({ ...defaults(), confirmed: true });
  assert.deepEqual(configured.scans, []);
  assert.deepEqual(configured.points, []);
});

console.log(`${passed} passed, ${failed} failed`);
process.exitCode = failed ? 1 : 0;
