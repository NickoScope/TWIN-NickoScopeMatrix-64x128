// Check web/panel.html in a real browser, both review findings of 2026-09-29:
// - the LED face is composited as light, the way hub75/src/render.rs makes it: the GOB's share
//   leaves the dot, dot and halo add in linear light, sRGB-encoded once. The expected pixels are
//   computed here from the same formulas, so a lit area that clips or a colour that shifts fails;
// - dragging the knob turns it without closing its switch (GPIO0), and a click holds the switch
//   150 ms: longer than the firmware's 40 ms debounce, shorter than its 500 ms click limit
//   (control.cpp, CTRL_SW_DEBOUNCE_MIN_MS and kSwShortMaxMs).
// Light frames go in through a captured WebSocket and pixels come back from the canvas; the knob
// gets real CDP mouse and key input. No emulator and no firmware. `?webgl=0` checks the canvas 2D
// fallback. Run against an isolated headless Chrome (README "CPU sampling" has the launch line)
// and `python3 -m http.server --directory web PORT`:
//   node tools/browser-benchmark/check-panel-page.mjs http://127.0.0.1:PORT/panel.html [DEBUG_PORT]
import assert from 'node:assert/strict';
const [url, port = '9228'] = process.argv.slice(2);
if (!url) throw Error('Usage: node check-panel-page.mjs PAGE_URL [DEBUG_PORT]');
const version = await (await fetch(`http://127.0.0.1:${port}/json/version`)).json();
assert.ok(version['User-Agent'].includes('HeadlessChrome/'), 'Use isolated headless Chrome: visible-page timers are required without taking user focus');
const socket = new WebSocket(version.webSocketDebuggerUrl);
await new Promise(resolve => socket.onopen = resolve);
let nextId = 0;
const pending = new Map();
socket.onmessage = ({data}) => {
  const message = JSON.parse(data);
  if (!message.id) return;
  const task = pending.get(message.id); pending.delete(message.id);
  message.error ? task.reject(Error(JSON.stringify(message.error))) : task.resolve(message.result);
};
const send = (method, params = {}, sessionId) => new Promise((resolve, reject) => {
  const id = ++nextId; pending.set(id, {resolve, reject});
  socket.send(JSON.stringify({id, method, params, sessionId}));
});
const sleep = ms => new Promise(r => setTimeout(r, ms));

// The page's model, mirrored: CSS Color 4 gam_sRGB, the room black #040405 as linear light, the
// dot and the gaussian halo normalised over its 7x7 taps (render.rs render_panel_into).
const W = 128, H = 64, DOT_R = 0.36, SIGMA = 0.9, R = Math.ceil(3 * SIGMA);
const BG = [4, 4, 5].map(v => v / 255 / 12.92);
const enc = v => { v = Math.min(1, Math.max(0, v)); return Math.round(255 * (v > 0.0031308 ? 1.055 * v ** (1 / 2.4) - 0.055 : 12.92 * v)); };
function expected(field, glow, px, py, p, dots = true) {
  const qx = (px + 0.5) / p, qy = (py + 0.5) / p, cx = Math.floor(qx), cy = Math.floor(qy), ux = qx - cx - 0.5, uy = qy - cy - 0.5;
  const cov = dots ? Math.min(1, Math.max(0, (DOT_R - Math.hypot(ux, uy)) * p + 0.5)) : 1;
  const own = field(cx, cy), acc = [0, 0, 0]; let sum = 0;
  for (let j = -R; j <= R; j++) for (let i = -R; i <= R; i++) {
    const w = Math.exp(-((ux - i) ** 2 + (uy - j) ** 2) / (2 * SIGMA * SIGMA)); sum += w;
    const x = cx + i, y = cy + j;
    if (x >= 0 && x < W && y >= 0 && y < H) field(x, y).forEach((l, k) => acc[k] += w * l);
  }
  return [0, 1, 2].map(k => enc(BG[k] + (1 - glow) * cov * own[k] + (glow > 0 ? glow * acc[k] / sum : 0)));
}
const q = v => Math.round(Math.min(1, v) * 65535) / 65535;   // what the u16 wire carries

let targetId;
const checks = [];
try {
  ({targetId} = await send('Target.createTarget', {url: 'about:blank', background: false}));
  const {sessionId} = await send('Target.attachToTarget', {targetId, flatten: true});
  const evaluate = async expression => {
    const result = await send('Runtime.evaluate', {expression, returnByValue: true, awaitPromise: true}, sessionId);
    if (result.exceptionDetails) throw Error(JSON.stringify(result.exceptionDetails));
    return result.result.value;
  };
  await send('Page.enable', {}, sessionId);
  await send('Emulation.setDeviceMetricsOverride', {width: 1400, height: 1000, deviceScaleFactor: 1, mobile: false}, sessionId);
  await send('Page.addScriptToEvaluateOnNewDocument', {source: `
    window.sent = [];
    window.WebSocket = class { constructor() { this.readyState = 1; window.fakeWs = this; setTimeout(() => this.onopen && this.onopen()); }
      send(d) { window.sent.push({...JSON.parse(d), at: performance.now()}); } close() {} };
    window.pushLight = (fn) => {
      const W = 128, H = 64, b = new ArrayBuffer(5 + W * H * 6), v = new DataView(b);
      v.setUint8(0, 5); v.setUint16(1, W, true); v.setUint16(3, H, true);
      for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) { const c = fn(x, y);
        for (let k = 0; k < 3; k++) v.setUint16(5 + ((y * W + x) * 3 + k) * 2, Math.round(Math.min(1, c[k]) * 65535), true); }
      window.fakeWs.onmessage({data: b});
      return new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));
    };
    window.pixels = (pts) => { const c = document.getElementById('panel'), t = document.createElement('canvas');
      t.width = c.width; t.height = c.height; const g = t.getContext('2d'); g.drawImage(c, 0, 0);
      return pts.map(([x, y]) => [...g.getImageData(x, y, 1, 1).data.slice(0, 3)]); };
    if (location.search.includes('webgl=0')) { const get = HTMLCanvasElement.prototype.getContext;
      HTMLCanvasElement.prototype.getContext = function (type, ...a) { return type === 'webgl2' ? null : get.call(this, type, ...a); }; }
  `}, sessionId);
  const load = async u => {
    await send('Page.navigate', {url: u}, sessionId);
    for (let i = 0; i < 100 && !await evaluate('!!window.fakeWs && document.readyState === "complete"'); i++) await sleep(20);
    assert.ok(await evaluate('!!window.fakeWs && document.readyState === "complete"'), 'page initialization timeout');
    await sleep(100);
  };
  const set = (id, value) => evaluate(`(() => { const e = document.getElementById('${id}');
    if (e.type === 'checkbox') e.checked = ${value}; else e.value = '${value}'; e.dispatchEvent(new Event('input')); })()`);
  // A field, its samples (LED centre, and the corner between four dots) and the expected pixels.
  const face = async (fieldSrc, pts) => {
    const field = eval(fieldSrc);
    await evaluate(`pushLight(${fieldSrc})`);
    const p = await evaluate('document.getElementById("panel").width') / W;
    const at = pts.map(([kind, x, y]) => kind === 'dot' ? [Math.floor((x + 0.5) * p), Math.floor((y + 0.5) * p)] : [Math.floor((x + 1) * p), Math.floor((y + 1) * p)]);
    return {got: await evaluate(`pixels(${JSON.stringify(at)})`), at, p, field: (x, y) => field(x, y).map(q)};
  };
  const near = (got, want, tol, what) => assert.ok(got.every((g, k) => Math.abs(g - want[k]) <= tol), `${what}: got ${got}, want ${want} ±${tol}`);

  // ---- WebGL2: linear light
  await load(url);
  assert.equal(await evaluate('!!document.getElementById("panel").getContext("webgl2")'), true, 'the page draws with WebGL2');
  const glow = 0.55;   // the page's default
  for (const [name, src] of [
    ['white at brightness 255 (2626 clocks)', '() => [0.892, 0.892, 0.892]'],
    ['orange fill (255,128,0)', '() => [0.892, 0.207, 0]'],
    ['code 193 (1478 clocks)', '() => [0.502, 0.502, 0.502]'],
    ['code 194 (1134 clocks)', '() => [0.385, 0.385, 0.385]'],
  ]) {
    const {got, at, p, field} = await face(src, [['dot', 64, 32], ['gap', 64, 32]]);
    near(got[0], expected(field, glow, ...at[0], p), 1, `${name}, dot centre`);
    near(got[1], expected(field, glow, ...at[1], p), 1, `${name}, between dots`);
    near(got[0], field(64, 32).map((l, k) => enc(l + BG[k])), 1, `${name}: a lit area stays at its light`);
  }
  checks.push('uniform fields keep their light', 'orange stays orange', '193/194 dip visible');
  {
    const src = '(x, y) => x === 64 && y === 32 ? [1, 1, 1] : [0, 0, 0]';
    const {got, at, p, field} = await face(src, [['dot', 64, 32], ['dot', 65, 32], ['dot', 66, 33], ['gap', 64, 32]]);
    got.forEach((g, i) => near(g, expected(field, glow, ...at[i], p), 1, `one lit LED, sample ${i}`));
    assert.ok(got[1][0] > 4 && got[1][0] < got[0][0], 'the halo reaches the next dot, dimmer');
    checks.push('halo of one LED matches the gaussian');
  }
  await set('glow', 0);
  {
    const {got, at, p, field} = await face('() => [0.892, 0.892, 0.892]', [['dot', 10, 10], ['gap', 10, 10]]);
    near(got[0], expected(field, 0, ...at[0], p), 1, 'glow 0, dot'); near(got[1], [4, 4, 5], 1, 'glow 0, room black between dots');
  }
  await set('glow', 0.55); await set('dots', false);
  {
    const {got, at, p, field} = await face('() => [0.892, 0.207, 0]', [['dot', 10, 10], ['gap', 10, 10]]);
    near(got[0], expected(field, glow, ...at[0], p, false), 1, 'no dots, cell'); near(got[1], got[0], 1, 'no dots: the cell is uniform');
  }
  await set('dots', true); await set('exposure', 2);
  {
    const {got, at, p, field} = await face('() => [0.3, 0.3, 0.3]', [['dot', 10, 10]]);
    near(got[0], expected((x, y) => field(x, y).map(l => 2 * l), glow, ...at[0], p), 1, 'exposure 2');
  }
  await set('exposure', 1);
  checks.push('glow 0', 'dots off', 'exposure');

  // ---- the knob, with real mouse and key input
  const knob = await evaluate(`(() => { const k = document.getElementById('knob'); k.scrollIntoView({block: 'center'});
    const r = k.getBoundingClientRect(); return {x: r.left + r.width / 2, y: r.top + r.height / 2}; })()`);
  const mouse = (type, dy = 0) => send('Input.dispatchMouseEvent', {type, x: knob.x, y: knob.y + dy, button: 'left',
    buttons: type === 'mouseReleased' ? 0 : 1, clickCount: type === 'mouseMoved' ? 0 : 1}, sessionId);
  const take = async () => { const s = await evaluate('window.sent.splice(0)'); return s.filter(m => m.t === 'knob' || m.t === 'knobpress'); };
  await take();
  const presses = s => s.filter(m => m.t === 'knobpress');
  const holds = s => { const p = presses(s); const out = []; for (let i = 0; i + 1 < p.length; i += 2) { assert.deepEqual([p[i].v, p[i + 1].v], ['1', '0']); out.push(p[i + 1].at - p[i].at); } return out; };

  await mouse('mousePressed'); await mouse('mouseReleased'); await sleep(450);
  let s = await take();
  assert.deepEqual(s.map(m => m.t + m.v), ['knobpress1', 'knobpress0'], 'a click is one press');
  assert.ok(holds(s).every(h => h >= 145 && h < 400), `the press holds past the 40 ms debounce and under the 500 ms click: ${holds(s)}`);
  checks.push('click = one 150 ms press');

  await mouse('mousePressed'); await sleep(300);
  for (const dy of [10, 30, 60, 90]) { await mouse('mouseMoved', dy); await sleep(30); }
  await mouse('mouseReleased', 90); await sleep(450);
  s = await take();
  assert.deepEqual(s.map(m => m.t + (m.d || m.v)), ['knob1', 'knob1', 'knob1'], 'a drag turns three clicks and never presses the switch');
  await mouse('mousePressed'); await sleep(30);
  for (const dy of [-30, -60]) { await mouse('mouseMoved', dy); await sleep(30); }
  await mouse('mouseReleased', -60); await sleep(450);
  s = await take();
  assert.deepEqual(s.map(m => m.t + (m.d || m.v)), ['knob-1', 'knob-1'], 'a quick drag the other way: turns only');
  checks.push('drag turns without pressing');

  await mouse('mousePressed'); await sleep(1200);
  assert.equal(presses(await evaluate('window.sent')).length, 0, 'nothing is pressed while the pointer is still down');
  await mouse('mouseReleased'); await sleep(450);
  s = await take();
  assert.deepEqual(s.map(m => m.t + m.v), ['knobpress1', 'knobpress0'], 'a held pointer released in place is one click (the firmware folds LONG into a click)');

  await mouse('mousePressed'); await mouse('mouseReleased'); await sleep(40); await mouse('mousePressed'); await mouse('mouseReleased'); await sleep(900);
  s = presses(await take());
  assert.deepEqual(s.map(m => m.v), ['1', '0', '1', '0'], 'a double click is two presses');
  assert.ok(s[2].at - s[1].at >= 145, `the second press waits for the switch to settle up: ${s[2].at - s[1].at} ms`);
  checks.push('hold = one click', 'double click serialised');

  await mouse('mousePressed');
  await evaluate(`document.getElementById('knob').dispatchEvent(new PointerEvent('pointercancel', {pointerId: 1}))`);
  await mouse('mouseReleased'); await sleep(450);
  assert.deepEqual(await take(), [], 'a cancelled pointer never presses');

  await evaluate(`document.getElementById('knob').focus()`);
  const key = (type, autoRepeat = false) => send('Input.dispatchKeyEvent', {type, key: 'Enter', code: 'Enter', windowsVirtualKeyCode: 13, autoRepeat}, sessionId);
  await key('keyDown'); await key('keyDown', true); await key('keyDown', true); await key('keyUp'); await sleep(450);
  s = await take();
  assert.deepEqual(s.map(m => m.t + m.v), ['knobpress1', 'knobpress0'], 'Enter is one click; its auto-repeat adds none');
  assert.ok(holds(s).every(h => h >= 145 && h < 400));
  checks.push('cancel', 'Enter');

  // ---- canvas 2D fallback: encoded layers, but the dot gives up the glow's share
  await load(url + (url.includes('?') ? '&' : '?') + 'webgl=0');
  assert.ok(await evaluate('!!document.getElementById("panel").getContext("2d")'), 'the fallback draws with canvas 2D');
  {
    const {got, field} = await face('() => [0.892, 0.207, 0]', [['dot', 64, 32]]);
    const want = field(64, 32).map(enc);
    assert.ok(got[0][0] <= want[0] + 5 && got[0][1] <= want[1] + 5 && got[0][0] < 255, `2D fallback, orange dot centre: got ${got[0]}, want about ${want} (+4 room black)`);
    assert.ok(got[0][0] >= want[0] - 2 && got[0][1] >= want[1] - 2, `2D fallback keeps the level: got ${got[0]}, want ${want}`);
  }
  checks.push('2D fallback does not clip');
  console.log(JSON.stringify({passed: true, browser: version.Browser, checks, scope: 'actual panel.html drawing and knob listeners, captured transport; no firmware'}));
} finally {
  if (targetId) await send('Target.closeTarget', {targetId});
  socket.close();
}
