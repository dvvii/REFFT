// REFFT sampler: Bela p5.js GUI (global mode). Open http://bela.local/gui
// while refft-cpp runs.
//
// Bela → browser (render.cpp sendGui):
//   Bela.data.buffers[0]  board floats, indices in ST: the sampler, meters,
//                         then the Pepper panel and CV outs (shown, not
//                         controlled here)
//   Bela.data.buffers[1]  file list, chars: "listId\nname\nname..."
//   Bela.data.buffers[2]  waveform overview: [clipId, min0, max0, min1, max1, ...]
//   Bela.data.buffers[3]  last loader message, chars
//   Bela.data.buffers[4]  field spectrum: [eventLoHz, eventHiHz, dBFS × 42, event share × 42]
// Browser → Bela: Bela.control.send({address, value}), see onGuiControl().
//
// - A value being edited is drawn from a local copy for a moment, so the
//   board's echo, a board frame behind, doesn't make the control jump.
// - Buffers sent before this page connected are lost. The board carries the
//   ids of the current file list and clip; when ours differ, ask for /sync.
// - Widgets are immediate mode; a drag stays with the widget it started on.
// - p5 only hosts the sketch (setup, draw, input); drawing goes straight to
//   the canvas, see "drawing" below.

'use strict';

const ST = {
	clip: 0, running: 1, head: 2, start: 3, end: 4, gain: 5, loop: 6, xfade: 7,
	channel: 8, normalize: 9, seconds: 10, rate: 11, loader: 12, file: 13,
	listId: 14, channels: 15, peakDb: 16, analysed: 17, play: 18, underruns: 19,
	meters: 20, layer: 24, freeze: 25, fft: 26, bypass: 27, touched: 28,
	knobs: 29, picked: 45, pots: 61, cv: 69, medianDb: 77, normDb: 78, limitDb: 79,
	limitRDb: 80, size: 81,
};
const METERS = ['in L (drone)', 'in R (field)', 'out L (sampler)', 'out R (stamp)'];
const CHANNELS = ['mix', 'L', 'R'];

const CVS = ['drone lvl', 'field lvl', 'noisiness', 'brightness', 'onset gate',
	'events', 'wet lvl', 'drone bright'];
const CV_LABELS = CVS.map((name, c) => 'CV' + c + ' ' + name);

// The panel's knobs: label and the value as render.cpp maps it
const gainDb = (v) => {
	const g = 4 * v * v;
	return g < 1e-3 ? 'off' : fmtDb(20 * Math.log10(g));
};
const fmtDb = (db) => (db >= 0 ? '+' : '') + db.toFixed(1) + ' dB';
const fmtSec = (s) => s < 1 ? (1000 * s).toFixed(0) + ' ms' : s.toFixed(2) + ' s';
const pct = (v) => (100 * v).toFixed(0) + '%';
const fmtHz = (hz) => hz < 1000 ? hz.toFixed(0) + ' Hz' : (hz / 1000).toFixed(hz < 10000 ? 1 : 0) + ' kHz';
const knobHz = (v) => fmtHz(100 * Math.pow(128, v));
const KNOBS = [
	[
		['drone gain', gainDb],
		['field gain', gainDb],
		['whiten', pct],
		['contrast', (v) => '^' + (0.5 * Math.pow(8, v)).toFixed(2)],
		['floor only', pct],
		['smear', (v) => fmtSec(5 * v * v)],
		['tonality', pct],
		['wet', pct],
	],
	[
		['out R gain', (v) => v < 0.02 ? 'off' : fmtDb(-24 + 48 * v)],
		['attack', (v) => fmtSec(2 * v * v)],
		['floor speed', (v) => (0.5 + 30 * v * v).toFixed(1) + ' dB/s'],
		['event thresh', (v) => (3 + 27 * v).toFixed(1) + ' dB'],
		['dry source', (v) => v < 0.02 ? 'field' : v > 0.98 ? 'drone' : 'drone ' + pct(v)],
		['CV smooth', (v) => fmtSec(0.001 + 0.5 * v * v)],
		['event low', knobHz],
		['event high', knobHz],
	],
];
const KNOB_LABELS = KNOBS.map((layer, l) => layer.map((def, k) => (l ? 'B' : 'A') + k + ' ' + def[0]));
const HOLD_MS = 600;
const PAD = 12;
const grey = (v) => 'rgb(' + v + ',' + v + ',' + v + ')';
const C = {
	bg: grey(22), panel: grey(32), line: grey(60), text: grey(210), dim: grey(120), ink: grey(20),
	rowLive: grey(40), rowIdle: grey(28), rowHover: grey(48), btn: grey(44), btnHover: grey(58), btnEdge: grey(70),
	seg: grey(40), thumb: grey(230), shade: 'rgba(0,0,0,0.47)', band: 'rgba(79,193,176,0.12)',
	accent: '#4fc1b0', region: 'rgba(79,193,176,0.16)', wave: '#8fa3b0', head: '#f0c060', warn: '#e06c75',
};
// spectrum bars, C.wave to C.head by event share
const RAMP = Array.from({ length: 33 }, (_, i) => {
	const t = i / 32, mix = (a, b) => Math.round(a + (b - a) * t);
	return 'rgb(' + mix(0x8f, 0xf0) + ',' + mix(0xa3, 0xc0) + ',' + mix(0xb0, 0x60) + ')';
});
const clamp = (v, lo, hi) => v < lo ? lo : v > hi ? hi : v;

// --- mirrored state ---------------------------------------------------------

let board = null; // latest status buffer
let statusTs = 0;
let files = [];
let filesId = -1;
let peaks = null;
let peaksId = -1;
let message = '';
let spectrum = null;
const counts = {};
const local = {};
let lastSyncAsk = -1e9;

function send(address, value) {
	try {
		Bela.control.send({ address: address, value: value });
	} catch (e) {
		// not connected yet
	}
}

function val(key) {
	const l = local[key];
	if (l && millis() - l.t < HOLD_MS)
		return l.v;
	return board ? board[ST[key]] : 0;
}

function setVal(key, address, value) {
	const l = local[key];
	local[key] = { v: value, t: millis() };
	if (!l || l.v !== value)
		send(address, value);
}

// char buffers arrive one byte per element; file names may be UTF-8
const utf8 = new TextDecoder();
function decodeText(b) {
	return utf8.decode(Uint8Array.from(b, (c) => c.charCodeAt(0)));
}

function fresh(i) {
	const b = Bela.data.buffers[i];
	if (!b || b.count === counts[i])
		return null;
	counts[i] = b.count;
	dirty = true;
	return b;
}

function readBuffers() {
	if (!Bela.data || !Bela.data.buffers)
		return;
	let b;
	if ((b = fresh(0)) && b.length >= ST.size) {
		diagStatus(b);
		board = b;
		statusTs = b.ts;
	}
	if ((b = fresh(1))) {
		const lines = decodeText(b).split('\n');
		filesId = +lines[0];
		files = lines.slice(1).filter((s) => s.length);
	}
	if ((b = fresh(2))) {
		peaksId = b[0];
		peaks = b;
		waveDirty = true;
	}
	if ((b = fresh(3)))
		message = decodeText(b);
	if ((b = fresh(4)))
		spectrum = b;

	const stale = !board
		|| board[ST.listId] !== filesId
		|| (board[ST.clip] > 0 && board[ST.clip] !== peaksId);
	if (stale && millis() - lastSyncAsk > 1000) {
		lastSyncAsk = millis();
		++diag.syncs;
		send('/sync', 1);
	}
}

function connected() {
	return board && performance.now() - statusTs < 1000;
}

// --- diagnostics --------------------------------------------------------------

// Tap the title for a line of diagnostics, double-tap it to zero the load
// count: how many times the sketch has loaded (Bela's page rebuilds it on
// every reconnect), the longest gaps between frames and between status
// buffers, the status rate, /sync requests, and the canvases and iframes in
// the page. Every 5 s a summary goes to localStorage, so after a reload, a
// crash included, the line also shows how the previous load ended.
const DIAG_KEY = 'refft-diag';
const diag = { show: false, loads: 0, since: 0, prev: null, last: null,
	windowStart: 0, lastFrame: 0, frameMax: 0, statusCount: -1, statusTs: 0, statusMax: 0, statusN: 0, syncs: 0 };

function stored(fn) {
	try {
		return fn();
	} catch (e) {
		return null; // no storage: the line still works for this load
	}
}

function diagSave() {
	stored(() => localStorage.setItem(DIAG_KEY, JSON.stringify(
		{ show: diag.show, loads: diag.loads, since: diag.since, last: diag.last })));
}

function diagLoad() {
	const saved = stored(() => JSON.parse(localStorage.getItem(DIAG_KEY))) || {};
	diag.show = !!saved.show;
	diag.loads = (saved.loads || 0) + 1;
	diag.since = saved.since || Date.now();
	diag.prev = saved.last || null;
	diag.windowStart = performance.now();
	diagSave();
}

function diagStatus(b) {
	if (diag.statusCount >= 0) {
		diag.statusN += b.count - diag.statusCount;
		diag.statusMax = Math.max(diag.statusMax, b.ts - diag.statusTs);
	}
	diag.statusCount = b.count;
	diag.statusTs = b.ts;
}

function diagFrame() {
	const now = performance.now();
	if (diag.lastFrame)
		diag.frameMax = Math.max(diag.frameMax, now - diag.lastFrame);
	diag.lastFrame = now;
	if (now - diag.windowStart < 5000)
		return;
	const parentDoc = window.parent !== window ? stored(() => window.parent.document) : null;
	diag.last = {
		t: Date.now(), frameMax: Math.round(diag.frameMax), statusMax: Math.round(diag.statusMax), status: diag.statusN,
		syncs: diag.syncs, canvases: document.querySelectorAll('canvas').length,
		iframes: parentDoc ? parentDoc.querySelectorAll('iframe').length : -1,
	};
	diag.frameMax = diag.statusMax = diag.statusN = diag.syncs = 0;
	diag.windowStart = now;
	diagText = makeDiagText();
	diagSave();
	dirty = true;
}

// rebuilt when it changes, not per frame
let diagText = '';

function makeDiagText() {
	const hhmm = (t) => new Date(t).toTimeString().slice(0, 5);
	const fmt = (d) => d ? 'frame gap ' + d.frameMax + ' ms · status gap ' + d.statusMax + ' ms, ' + d.status
		+ '/5 s · syncs ' + d.syncs + ' · canvases ' + d.canvases + ' · iframes ' + d.iframes : '…';
	let s = 'loads ' + diag.loads + ' since ' + hhmm(diag.since) + ' · ' + fmt(diag.last);
	if (diag.prev)
		s += '   |   before this load (' + hhmm(diag.prev.t) + '): ' + fmt(diag.prev);
	return s;
}

// --- input ------------------------------------------------------------------

// tap: a press released without moving (touch), so a swipe over the file
// list scrolls it instead of loading the file under the finger
const mouse = { pressed: false, released: false, double: false, wheel: 0, touch: false, tap: false };
let drag = null; // { id, ... } while a widget owns the mouse

// Pointer position in canvas pixels. Input comes from Pointer Events on the
// canvas, which cover mouse, touch and pencil alike; p5's mouseX and touch
// callbacks don't follow a finger on iPad.
let ptrX = -1, ptrY = -1;
const ptr = { id: null, clearHover: false, x0: 0, y0: 0, lastY: 0, moved: false, scroll: 0, prevTapMs: -1e9, prevTapX: 0, prevTapY: 0 };

function setPtr(e) {
	const r = e.target.getBoundingClientRect();
	ptrX = e.clientX - r.left;
	ptrY = e.clientY - r.top;
	dirty = true;
}

// End the held pointer, if any: the next frame drops its drag
function release() {
	if (null === ptr.id) return;
	ptr.id = null;
	mouse.released = true;
	dirty = true;
	if (mouse.touch) ptr.clearHover = true; // no hover once the finger has lifted
}

function attachPointer(el) {
	el.style.touchAction = 'none';
	el.addEventListener('pointerdown', (e) => {
		// one pointer at a time. A new primary pointer means every earlier
		// touch has ended, so one still held is stale: iPadOS can swallow its
		// pointerup and pointercancel (system gestures, lock, app switch)
		if (null !== ptr.id) {
			if (!e.isPrimary) return;
			drag = null;
		}
		ptr.id = e.pointerId;
		try { el.setPointerCapture(e.pointerId); } catch (err) {}
		setPtr(e);
		mouse.pressed = true;
		mouse.touch = 'mouse' !== e.pointerType;
		// a second tap within 350 ms at the same spot is a double click
		if (mouse.touch && performance.now() - ptr.prevTapMs < 350 && Math.hypot(ptrX - ptr.prevTapX, ptrY - ptr.prevTapY) < 30)
			mouse.double = true;
		ptr.x0 = ptrX; ptr.y0 = ptr.lastY = ptrY; ptr.moved = false; ptr.scroll = 0;
		e.preventDefault();
	});
	el.addEventListener('pointermove', (e) => {
		if (null !== ptr.id && e.pointerId !== ptr.id) return;
		setPtr(e);
		if (null === ptr.id || 'mouse' === e.pointerType) return;
		if (Math.hypot(ptrX - ptr.x0, ptrY - ptr.y0) > 10) ptr.moved = true;
		// a swipe that no widget owns scrolls, one list row per 22 px
		if (!drag) {
			ptr.scroll += ptr.lastY - ptrY;
			while (Math.abs(ptr.scroll) >= 22) {
				mouse.wheel += Math.sign(ptr.scroll);
				ptr.scroll -= 22 * Math.sign(ptr.scroll);
			}
		}
		ptr.lastY = ptrY;
		e.preventDefault();
	});
	// only a real pointerup can be a tap; a cancelled or lost pointer just ends
	const up = (e) => {
		if (e.pointerId !== ptr.id) return;
		setPtr(e);
		const tap = 'pointerup' === e.type && !ptr.moved;
		release();
		mouse.tap = tap;
		if (mouse.touch && tap) { ptr.prevTapMs = performance.now(); ptr.prevTapX = ptrX; ptr.prevTapY = ptrY; }
	};
	el.addEventListener('pointerup', up);
	el.addEventListener('pointercancel', up);
	el.addEventListener('lostpointercapture', up);
	// the page hidden or unfocused mid-touch may never see the touch end
	document.addEventListener('visibilitychange', () => { if (document.hidden) release(); });
	window.addEventListener('blur', release);
	el.addEventListener('pointerleave', (e) => { if ('mouse' === e.pointerType && null === ptr.id) { ptrX = ptrY = -1; dirty = true; } });
	el.addEventListener('dblclick', () => { if (!mouse.touch) { mouse.double = true; dirty = true; } });
	el.addEventListener('touchstart', (e) => e.preventDefault(), { passive: false });
	el.addEventListener('touchmove', (e) => e.preventDefault(), { passive: false });
}

function mouseWheel(e) { mouse.wheel += e.delta; dirty = true; return false; }

function over(x, y, w, h) {
	return ptrX >= x && ptrX < x + w && ptrY >= y && ptrY < y + h;
}

function keyPressed() {
	dirty = true;
	if (' ' === key) setVal('play', '/play', val('play') > 0.5 ? 0 : 1);
	else if ('Enter' === key || 'Home' === key) send('/retrigger', 1);
	else if ('l' === key) setVal('loop', '/loop', val('loop') > 0.5 ? 0 : 1);
	else return true;
	return false;
}

// --- drawing ------------------------------------------------------------------

// Everything is drawn straight on the canvas's 2D context, not through p5.
// p5's text() in a box measures the font by adding elements to the page and
// reading back their layout, after every textSize(); with ~50 boxed labels a
// frame that was 3000 page layouts a second and a stream of discarded DOM
// nodes, which on iPad grew into garbage collector stalls until Safari
// killed the page. p5's fill() and stroke() also parse colour strings on
// every call. Here colours are fixed strings, context state is set only
// when it changes, and text is one native fillText per line.
let ctx = null;
let cnvEl = null;
const cur = { fill: null, stroke: null, lw: 0, cap: null, font: null, align: null, base: null, cursor: null };
const FONTS = {};
let charW = 0.6; // advance of one monospace character per px of size, measured in setup()

function fillC(c) {
	if (cur.fill !== c) ctx.fillStyle = cur.fill = c;
}

function strokeC(c, lw, cap) {
	if (cur.stroke !== c) ctx.strokeStyle = cur.stroke = c;
	if (cur.lw !== lw) ctx.lineWidth = cur.lw = lw;
	if (cur.cap !== cap) ctx.lineCap = cur.cap = cap;
}

function textC(size, c, align, base) {
	const f = FONTS[size] || (FONTS[size] = size + 'px monospace');
	if (cur.font !== f) ctx.font = cur.font = f;
	if (cur.align !== align) ctx.textAlign = cur.align = align;
	if (cur.base !== base) ctx.textBaseline = cur.base = base;
	fillC(c);
}

// The canvas resets its context state when resized, so forget what was set
function beginFrame() {
	ctx = drawingContext;
	for (const k in cur) cur[k] = null;
	const d = pixelDensity();
	ctx.setTransform(d, 0, 0, d, 0, 0);
}

function fillR(x, y, w, h, r, c) {
	if (w <= 0 || h <= 0) return;
	fillC(c);
	if (r && ctx.roundRect) {
		ctx.beginPath();
		ctx.roundRect(x, y, w, h, Math.min(r, w / 2, h / 2));
		ctx.fill();
	} else
		ctx.fillRect(x, y, w, h);
}

function rule(x0, y0, x1, y1, c, lw) {
	strokeC(c, lw, 'round');
	ctx.beginPath();
	ctx.moveTo(x0, y0);
	ctx.lineTo(x1, y1);
	ctx.stroke();
}

function dot(x, y, d, c) {
	fillC(c);
	ctx.beginPath();
	ctx.arc(x, y, d / 2, 0, 2 * Math.PI);
	ctx.fill();
}

function txt(s, x, y, size, c, align, base) {
	textC(size, c, align, base);
	ctx.fillText(s, x, y);
}

// Lines of s broken at spaces to fit maxW, at most maxLines of them, as
// p5's text box laid them out; monospace, so a width is a character count.
// Cached, since most labels never change.
const wraps = new Map();

function wrap(s, maxW, size, maxLines) {
	const key = size + '|' + Math.floor(maxW) + '|' + maxLines + '|' + s;
	let lines = wraps.get(key);
	if (lines) return lines;
	const n = Math.max(1, Math.floor(maxW / (size * charW)));
	lines = [];
	let rest = s;
	while (rest.length && lines.length < maxLines) {
		if (rest.length <= n) {
			lines.push(rest);
			break;
		}
		let cut = rest.lastIndexOf(' ', n);
		if (cut <= 0) cut = n;
		lines.push(rest.slice(0, cut));
		rest = rest.slice(cut).replace(/^ +/, '');
	}
	if (wraps.size > 500) wraps.clear();
	wraps.set(key, lines);
	return lines;
}

// s wrapped into a box; align 'left', 'center' or 'right', valign 'top' or 'middle'
function txtBox(s, x, y, w, h, size, c, align, valign) {
	const lead = 1.25 * size;
	const lines = wrap(s, w, size, Math.max(1, Math.floor(h / lead)));
	const ax = 'left' === align ? x : 'right' === align ? x + w : x + w / 2;
	let ly = 'top' === valign ? y : y + (h - lines.length * lead) / 2;
	textC(size, c, align, 'top');
	for (let i = 0; i < lines.length; ++i, ly += lead)
		ctx.fillText(lines[i], ax, ly);
}

function setCursor(c) {
	if (cur.cursor !== c) cnvEl.style.cursor = cur.cursor = c;
}

// --- p5 lifecycle -----------------------------------------------------------

let wave = null;     // cached waveform image, a canvas outside the page
let waveDirty = true;
let fileScroll = 0;
// Redraw only when a buffer arrived, the pointer or keys did something, or
// at least every IDLE_MS (timer, timeouts); the board sends at 30 Hz.
let dirty = true;
let lastDraw = -1e9;
const IDLE_MS = 250;

function setup() {
	const cnv = createCanvas(windowWidth, windowHeight);
	cnvEl = cnv.elt;
	attachPointer(cnvEl);
	frameRate(30);
	ctx = drawingContext;
	ctx.font = '100px monospace';
	charW = ctx.measureText('0123456789').width / 1000 || charW;
	diagLoad();
	diagText = makeDiagText();
	// no browser scrolling, pinch zoom or double-tap zoom over the GUI: the
	// window-level touch listeners are passive, so returning false can't stop them
	document.documentElement.style.touchAction = 'none';
	document.body.style.touchAction = 'none';
	document.body.style.overscrollBehavior = 'none';
	document.body.style.webkitUserSelect = 'none';
	document.body.style.webkitTouchCallout = 'none';
}

function windowResized() {
	resizeCanvas(windowWidth, windowHeight);
	waveDirty = true;
	dirty = true;
}

// An exception out of draw() stops p5's loop and freezes the page, so one
// bad frame is logged (once per distinct message) and the next one drawn.
let lastError = '';

function draw() {
	diagFrame();
	try {
		readBuffers();
		if (!dirty && millis() - lastDraw < IDLE_MS)
			return; // nothing new: the canvas keeps the last frame
		dirty = false;
		lastDraw = millis();
		beginFrame();
		drawFrame();
	} catch (e) {
		if (String(e) !== lastError) {
			lastError = String(e);
			console.error(e);
		}
	}

	if (mouse.released)
		drag = null;
	mouse.pressed = mouse.released = mouse.double = mouse.tap = false;
	mouse.wheel = 0;
	if (ptr.clearHover && null === ptr.id) {
		ptrX = ptrY = -1;
		dirty = true; // draw once more without the hover
	}
	ptr.clearHover = false;
}

function drawFrame() {
	fillR(0, 0, width, height, 0, C.bg);

	const narrow = width < 720;
	let y = drawHeader(PAD);
	const listW = narrow ? width - 2 * PAD : Math.min(300, width * 0.3);
	const mainX = narrow ? PAD : PAD + listW + PAD;
	const mainW = width - mainX - PAD;

	let my = drawWaveform(mainX, y, mainW, narrow ? 140 : 200);
	my = drawTransport(mainX, my + PAD, mainW);
	my = drawSliders(mainX, my + PAD, mainW);
	my = drawMeters(mainX, my + PAD, mainW);
	my = drawPanel(mainX, my + PAD, mainW);
	my = drawCv(mainX, my + PAD, mainW);
	my = drawSpectrum(mainX, my + PAD, mainW);
	my = drawTimer(mainX, my + PAD, mainW);
	if (narrow)
		drawFiles(PAD, my + PAD, listW, Math.max(160, height - my - 2 * PAD));
	else
		drawFiles(PAD, y, listW, height - y - PAD);
}

// --- header -----------------------------------------------------------------

function drawHeader(y) {
	const h = 34, cy = y + h / 2;
	txt('REFFT', PAD, cy, 16, C.text, 'left', 'middle');
	const on = connected();
	dot(PAD + 70, cy, 9, on ? C.accent : C.warn);
	txt(on ? 'connected' : 'waiting for refft-cpp', PAD + 81, cy, 11, C.dim, 'left', 'middle');

	if (over(PAD, y, 60, h)) {
		if (mouse.double) {
			diag.loads = 1;
			diag.since = Date.now();
			diagText = makeDiagText();
			diagSave();
		} else if (mouse.touch ? mouse.tap : mouse.pressed) {
			diag.show = !diag.show;
			diagSave();
		}
	}
	if (diag.show)
		txtBox(diagText, PAD + 220, y, width - 2 * PAD - 220, h, 10, C.dim, 'right', 'middle');
	else
		txt('sampler → out L · patch it into in R to stamp it', width - PAD, cy, 11, C.dim, 'right', 'middle');
	return y + h + PAD;
}

// --- waveform ---------------------------------------------------------------

// One offscreen canvas, redrawn in place and never added to the page
function renderWave(w, h) {
	const d = pixelDensity();
	if (!wave)
		wave = document.createElement('canvas');
	if (wave.width !== Math.round(w * d) || wave.height !== Math.round(h * d)) {
		wave.width = Math.round(w * d);
		wave.height = Math.round(h * d);
	}
	const g = wave.getContext('2d');
	g.setTransform(d, 0, 0, d, 0, 0);
	g.fillStyle = C.panel;
	g.fillRect(0, 0, w, h);
	g.strokeStyle = C.wave;
	g.lineWidth = 1;
	g.beginPath();
	const bins = (peaks.length - 1) / 2;
	for (let x = 0; x < w; ++x) {
		const b0 = Math.floor(x * bins / w);
		const b1 = Math.max(b0 + 1, Math.floor((x + 1) * bins / w));
		let lo = 0, hi = 0;
		for (let b = b0; b < b1; ++b) {
			lo = Math.min(lo, peaks[1 + 2 * b]);
			hi = Math.max(hi, peaks[2 + 2 * b]);
		}
		g.moveTo(x + 0.5, h / 2 - hi * h * 0.48);
		g.lineTo(x + 0.5, h / 2 - lo * h * 0.48);
	}
	g.stroke();
	wave.cssW = w;
	wave.cssH = h;
	waveDirty = false;
}

function fmtTime(s) {
	const m = Math.floor(s / 60);
	const sec = s - 60 * m;
	return m + ':' + (sec < 10 ? '0' : '') + sec.toFixed(1);
}

function drawWaveform(x, y, w, h) {
	if (!board || board[ST.clip] <= 0) {
		fillR(x, y, w, h, 4, C.panel);
		txt(board && board[ST.loader] === 1 ? 'opening…' : 'no file loaded: pick one on the left', x + w / 2, y + h / 2, 12, C.dim, 'center', 'middle');
		return y + h;
	}
	if (peaks && peaksId === board[ST.clip]) {
		if (waveDirty || !wave || wave.cssW !== w || wave.cssH !== h)
			renderWave(w, h);
		ctx.drawImage(wave, x, y, w, h);
	} else {
		// the overview comes from a pass through the whole file (once per file)
		fillR(x, y, w, h, 4, C.panel);
		const p = board[ST.analysed];
		fillR(x, y + h - 4, p * w, 4, 0, C.line);
		txt('reading the file for its waveform and peak… ' + Math.floor(100 * p) + '%', x + w / 2, y + h / 2, 12, C.dim, 'center', 'middle');
	}

	let s = val('start'), e = val('end');
	const xs = x + s * w, xe = x + e * w;
	const seconds = board[ST.seconds];

	// interaction: drag an edge, drag out a new region, click to seek,
	// double-click for the whole clip
	const t = clamp((ptrX - x) / w, 0, 1);
	if (mouse.pressed && over(x, y, w, h)) {
		if (Math.abs(ptrX - xs) < 8) drag = { id: 'wave', mode: 'start' };
		else if (Math.abs(ptrX - xe) < 8) drag = { id: 'wave', mode: 'end' };
		else drag = { id: 'wave', mode: 'new', anchor: t, x0: ptrX, moved: false };
	}
	if (drag && drag.id === 'wave') {
		if ('start' === drag.mode) {
			s = Math.min(t, e - 0.001);
			setVal('start', '/start', s);
		} else if ('end' === drag.mode) {
			e = Math.max(t, s + 0.001);
			setVal('end', '/end', e);
		} else {
			if (Math.abs(ptrX - drag.x0) > 4)
				drag.moved = true;
			if (drag.moved) {
				s = Math.min(drag.anchor, t);
				e = Math.max(drag.anchor, t);
				setVal('start', '/start', s);
				setVal('end', '/end', e);
			}
			if (mouse.released && !drag.moved)
				send('/seek', t);
		}
	}
	if (mouse.double && over(x, y, w, h)) {
		setVal('start', '/start', 0);
		setVal('end', '/end', 1);
		s = 0; e = 1;
	}

	const rs = x + s * w, re = x + e * w;
	fillR(x, y, rs - x, h, 0, C.shade);
	fillR(re, y, x + w - re, h, 0, C.shade);
	fillR(rs, y, re - rs, h, 0, C.region);
	rule(rs, y, rs, y + h, C.accent, 2);
	rule(re, y, re, y + h, C.accent, 2);
	const hx = x + board[ST.head] * w;
	rule(hx, y, hx, y + h, C.head, 1.5);
	if (over(x, y, w, h) && !drag)
		setCursor(Math.abs(ptrX - rs) < 8 || Math.abs(ptrX - re) < 8 ? 'ew-resize' : 'crosshair');
	else if (!drag)
		setCursor('default');

	txt(fmtTime(s * seconds) + ' → ' + fmtTime(e * seconds) + '   (' + fmtTime((e - s) * seconds) + ')', x, y + h + 3, 10, C.dim, 'left', 'top');
	txt(fmtTime(board[ST.head] * seconds) + ' / ' + fmtTime(seconds), x + w, y + h + 3, 10, C.dim, 'right', 'top');
	return y + h + 16;
}

// --- transport --------------------------------------------------------------

const TOGGLES = [['loop', 'loop'], ['normalize', 'normalise']];

function drawTransport(x, y, w) {
	const h = 30;
	const playing = val('play') > 0.5;
	let cx = x;
	if (button(cx, y, 90, h, playing ? '■ stop' : '▶ play', playing))
		setVal('play', '/play', playing ? 0 : 1);
	cx += 94;
	if (button(cx, y, 70, h, '⟲ start', false))
		send('/retrigger', 1);
	cx += 74 + 10;
	for (let i = 0; i < TOGGLES.length; ++i) {
		const [key, label] = TOGGLES[i];
		const on = val(key) > 0.5;
		const bw = 14 + label.length * 8;
		if (button(cx, y, bw, h, label, on))
			setVal(key, '/' + key, on ? 0 : 1);
		cx += bw + 4;
	}
	cx += 10;
	const mono = board && board[ST.channels] < 2;
	txt('channel', cx, y + h / 2, 11, mono ? C.line : C.dim, 'left', 'middle');
	cx += 60;
	const ch = Math.round(val('channel'));
	for (let i = 0; i < CHANNELS.length; ++i)
		if (segment(cx + i * 40, y, 38, h, CHANNELS[i], i === ch, mono))
			setVal('channel', '/channel', i);
	cx += CHANNELS.length * 40 + 12;

	// file info and loader messages
	const c = board && board[ST.loader] === 2 ? C.warn : C.dim;
	let info = message;
	if (board && board[ST.clip] > 0)
		info += board[ST.medianDb] > -180
			? '   level ' + board[ST.medianDb].toFixed(0) + ' dBFS'
				+ (val('normalize') > 0.5 ? ', normalised ' + fmtDb(board[ST.normDb]) : '')
			: val('normalize') > 0.5 ? '   (normalised once read through)' : '';
	if (board && board[ST.limitDb] < -0.1)
		info += '   limiting ' + board[ST.limitDb].toFixed(1) + ' dB';
	if (board && board[ST.underruns] > 0)
		info += '   disk underruns: ' + board[ST.underruns];
	if (cx + 200 < x + w) {
		txtBox(info, cx, y, x + w - cx, h, 11, c, 'left', 'middle');
		return y + h;
	}
	txtBox(info, x, y + h + 6, w, 30, 11, c, 'left', 'top');
	return y + h + 30;
}

// --- sliders ----------------------------------------------------------------

function drawSliders(x, y, w) {
	const cols = w < 520 ? 1 : 2;
	const sw = (w - (cols - 1) * 2 * PAD) / cols;
	const px = (i) => x + (i % cols) * (sw + 2 * PAD);
	const py = (i) => y + Math.floor(i / cols) * 40;

	slider('gain', px(0), py(0), sw, 'gain', val('gain'),
		(v) => (v + 36) / 60, (t) => -36 + 60 * t,
		(v) => (v >= 0 ? '+' : '') + v.toFixed(1) + ' dB',
		(v) => setVal('gain', '/gain', v), 0);
	slider('xfade', px(1), py(1), sw, 'crossfade', val('xfade'),
		(v) => Math.log(v / 2) / Math.log(1000), (t) => 2 * Math.pow(1000, t),
		(v) => v < 1000 ? v.toFixed(0) + ' ms' : (v / 1000).toFixed(2) + ' s',
		(v) => setVal('xfade', '/xfade', v), 50);
	return py(1) + 34;
}

// Drag to set, shift for fine steps, wheel to nudge, double-click to reset.
function slider(id, x, y, w, label, value, toT, fromT, fmt, onSet, def) {
	const ty = y + 22;
	const t = clamp(toT(value), 0, 1);
	txt(label, x, y + 11, 11, C.dim, 'left', 'alphabetic');
	txt(fmt(value), x + w, y + 11, 11, C.text, 'right', 'alphabetic');
	fillR(x, ty - 2, w, 4, 2, C.line);
	fillR(x, ty - 2, t * w, 4, 2, C.accent);
	dot(x + t * w, ty, 12, C.thumb);

	const hit = over(x - 6, y, w + 12, 32);
	if (mouse.pressed && hit)
		drag = { id: id, x0: ptrX, t0: t };
	if (drag && drag.id === id) {
		const nt = keyIsDown(SHIFT)
			? drag.t0 + (ptrX - drag.x0) / w * 0.1
			: (ptrX - x) / w;
		onSet(fromT(clamp(nt, 0, 1)));
	}
	if (hit && mouse.wheel)
		onSet(fromT(clamp(t - Math.sign(mouse.wheel) * 0.01, 0, 1)));
	if (hit && mouse.double)
		onSet(def);
}

// --- meters -----------------------------------------------------------------

function drawMeters(x, y, w) {
	const n = METERS.length;
	const mw = (w - (n - 1) * PAD) / n;
	for (let m = 0; m < n; ++m) {
		const mx = x + m * (mw + PAD);
		const db = board ? board[ST.meters + m] : -120;
		const t = clamp((db + 60) / 60, 0, 1);
		let label = METERS[m];
		if (board && 3 === m && board[ST.limitRDb] < -0.1)
			label += ' · lim ' + board[ST.limitRDb].toFixed(0) + ' dB';
		txt(label, mx, y + 10, 10, C.dim, 'left', 'alphabetic');
		txt(db > -60 ? db.toFixed(0) + ' dB' : '–', mx + mw, y + 10, 10, C.dim, 'right', 'alphabetic');
		fillR(mx, y + 14, mw, 8, 2, C.panel);
		fillR(mx, y + 14, t * mw, 8, 2, db > -1 ? C.warn : db > -12 ? C.head : C.accent);
	}
	return y + 24;
}

// --- Pepper panel -------------------------------------------------------------

// Read only: buttons 1-4, then the knobs of both layers. The live layer is
// lit; a knob of it that is waiting for pickup shows where the physical knob
// is (orange tick) against the stored value it has to reach.
const CHIP_SHARE = [1, 1, 1.8, 1];
const KNOB_A0 = Math.PI * 0.75, KNOB_A1 = Math.PI * 2.25;

function chip(x, y, w, label, value, on) {
	fillR(x, y, w, 26, 4, on ? C.accent : C.panel);
	txt(label, x + 8, y + 13, 10, on ? C.ink : C.dim, 'left', 'middle');
	txt(value, x + w - 8, y + 13, 11, on ? C.ink : C.text, 'right', 'middle');
	return x + w + 6;
}

function drawPanel(x, y, w) {
	if (!board)
		return y;
	const layer = board[ST.layer];
	const unit = (w - 3 * 6) / 4.8;
	let cx = x;
	cx = chip(cx, y, CHIP_SHARE[0] * unit, '1 shift', layer > 0.5 ? 'layer B' : 'layer A', layer > 0.5);
	cx = chip(cx, y, CHIP_SHARE[1] * unit, '2 freeze', board[ST.freeze] > 0.5 ? 'frozen' : 'off', board[ST.freeze] > 0.5);
	cx = chip(cx, y, CHIP_SHARE[2] * unit, '3 FFT', board[ST.fft] + ' (' + (1000 * board[ST.fft] * 1.25 / 44100).toFixed(0) + ' ms)', false);
	chip(cx, y, CHIP_SHARE[3] * unit, '4 bypass', board[ST.bypass] > 0.5 ? 'bypassed' : 'off', board[ST.bypass] > 0.5);
	y += 26 + 8;

	const touched = Math.round(board[ST.touched]);
	const kw = w / 8;
	const rowH = 58;
	const r = Math.min(14, kw / 2 - 6);
	for (let l = 0; l < 2; ++l) {
		const live = Math.round(layer) === l;
		fillR(x, y, w, rowH, 4, live ? C.rowLive : C.rowIdle);
		for (let k = 0; k < 8; ++k) {
			const def = KNOBS[l][k];
			const cx = x + k * kw + kw / 2;
			const v = board[ST.knobs + 8 * l + k];
			const waiting = live && board[ST.picked + 8 * l + k] < 0.5;
			const hot = live && touched === k && !waiting;
			txtBox(KNOB_LABELS[l][k], x + k * kw + 2, y + 3, kw - 4, 12, 9, live ? C.dim : C.line, 'center', 'top');
			const cy = y + 30;
			strokeC(C.line, 3, 'butt');
			ctx.beginPath();
			ctx.arc(cx, cy, r, KNOB_A0, KNOB_A1);
			ctx.stroke();
			if (v > 0.002) {
				strokeC(hot ? C.head : live ? C.accent : C.wave, 3, 'butt');
				ctx.beginPath();
				ctx.arc(cx, cy, r, KNOB_A0, KNOB_A0 + v * (KNOB_A1 - KNOB_A0));
				ctx.stroke();
			}
			if (waiting) {
				const p = KNOB_A0 + board[ST.pots + k] * (KNOB_A1 - KNOB_A0);
				const c = Math.cos(p), s = Math.sin(p);
				rule(cx + c * (r - 5), cy + s * (r - 5), cx + c * (r + 3), cy + s * (r + 3), C.head, 2);
			}
			txt(waiting ? 'turn to ' + pct(v) : def[1](v), cx, y + rowH - 3, 10, waiting ? C.head : live ? C.text : C.dim, 'center', 'alphabetic');
		}
		y += rowH + 4;
	}
	return y - 4;
}

// The field (in R) per display band: level bars, lit by how much of the band
// counts as events; the event band (B6, B7) is the bright strip.
const SPECTRUM_TICKS = [100, 200, 500, 1000, 2000, 5000, 10000];
const SPECTRUM_TICK_LABELS = SPECTRUM_TICKS.map(fmtHz);

function drawSpectrum(x, y, w) {
	if (!spectrum || spectrum.length < 2 + 2 * 42)
		return y;
	const n = (spectrum.length - 2) / 2;
	const h = 80, lo = -100, hi = -20;
	const octaves = 7, f0 = 100;
	const xOf = (hz) => x + w * Math.log2(hz / f0) / octaves;
	fillR(x, y, w, h + 16, 4, C.panel);
	const bx0 = xOf(Math.max(f0, spectrum[0])), bx1 = xOf(Math.min(f0 * 128, spectrum[1]));
	fillR(bx0, y, bx1 - bx0, h, 0, C.band);
	rule(bx0, y, bx0, y + h, C.accent, 1);
	rule(bx1, y, bx1, y + h, C.accent, 1);
	const bw = w / n;
	for (let b = 0; b < n; ++b) {
		const db = spectrum[2 + b], ev = clamp(spectrum[2 + n + b], 0, 1);
		const t = clamp((db - lo) / (hi - lo), 0, 1);
		fillR(x + b * bw + 1, y + h - t * h, bw - 2, t * h, 0, RAMP[Math.round(32 * ev)]);
	}
	for (let i = 0; i < SPECTRUM_TICKS.length; ++i)
		txt(SPECTRUM_TICK_LABELS[i], xOf(SPECTRUM_TICKS[i]), y + h + 3, 9, C.dim, 'center', 'top');
	txt('in R spectrum · yellow = events · band ' + fmtHz(spectrum[0]) + '–' + fmtHz(spectrum[1]) + ' (B6, B7)', x + 6, y + 4, 9, C.dim, 'left', 'top');
	return y + h + 16;
}

// Performance timer: a stopwatch in the browser, independent of the board
// and the sampler. As tall as the spectrum block. Wall-clock time, since
// performance.now() can stand still while an iPad sleeps; kept in
// sessionStorage because Bela's GUI page reloads the sketch whenever its
// connection to the board comes back. Starts stopped in a new tab.
const TIMER_KEY = 'refft-timer';
const timer = loadTimer();

function loadTimer() {
	try {
		const t = JSON.parse(sessionStorage.getItem(TIMER_KEY));
		if (t && 'number' === typeof t.startMs && 'number' === typeof t.accMs)
			return { running: !!t.running, startMs: t.startMs, accMs: t.accMs };
	} catch (e) {
		// no storage: start afresh
	}
	return { running: false, startMs: 0, accMs: 0 };
}

function saveTimer() {
	try {
		sessionStorage.setItem(TIMER_KEY, JSON.stringify(timer));
	} catch (e) {
		// no storage: the timer still runs, it just won't survive a reload
	}
}

function timerMs() {
	return Math.max(0, timer.accMs + (timer.running ? Date.now() - timer.startMs : 0));
}

function drawTimer(x, y, w) {
	const h = 96;
	fillR(x, y, w, h, 4, C.panel);
	const bw = Math.min(130, w * 0.25), bh = (h - 3 * 8) / 2, bx = x + w - bw - 8;
	if (button(bx, y + 8, bw, bh, timer.running ? 'stop' : 'start', timer.running)) {
		if (timer.running) timer.accMs += Date.now() - timer.startMs;
		else timer.startMs = Date.now();
		timer.running = !timer.running;
		saveTimer();
	}
	if (button(bx, y + 16 + bh, bw, bh, 'reset', false)) {
		timer.accMs = 0;
		timer.startMs = Date.now();
		saveTimer();
	}
	const sec = Math.floor(timerMs() / 1000);
	const pad2 = (v) => String(v).padStart(2, '0');
	const label = pad2(Math.floor(sec / 3600)) + ':' + pad2(Math.floor(sec / 60) % 60) + ':' + pad2(sec % 60);
	const tw = bx - x - 16;
	const size = Math.floor(Math.min(h * 0.8, tw / (label.length * charW)));
	txt(label, x + 8 + tw / 2, y + h / 2 + 2, size, timer.running ? C.text : C.dim, 'center', 'middle');
	return y + h;
}

// CV outs 0-7 as 0-5 V bars
function drawCv(x, y, w) {
	if (!board)
		return y;
	const h = 46;
	const cw = w / 8;
	fillR(x, y, w, h + 18, 4, C.panel);
	for (let c = 0; c < 8; ++c) {
		const v = clamp(board[ST.cv + c], 0, 1);
		const bx = x + c * cw + 8, bw = cw - 16;
		txtBox(CV_LABELS[c], x + c * cw + 2, y + 3, cw - 4, 12, 9, C.dim, 'center', 'top');
		fillR(bx, y + 16, bw, h - 16, 2, C.bg);
		fillR(bx, y + h - v * (h - 16), bw, v * (h - 16), 2, 4 === c ? (v > 0.5 ? C.head : C.line) : C.accent);
		txt((5 * v).toFixed(2) + ' V', x + c * cw + cw / 2, y + h + 14, 10, C.text, 'center', 'alphabetic');
	}
	return y + h + 18;
}

// --- files ------------------------------------------------------------------

function drawFiles(x, y, w, h) {
	fillR(x, y, w, h, 4, C.panel);
	txt('files', x + 10, y + 16, 12, C.text, 'left', 'middle');
	if (button(x + w - 80, y + 4, 72, 24, 'rescan', false))
		send('/rescan', 1);

	const rowH = 22;
	const top = y + 34;
	const visible = Math.floor((h - 40) / rowH);
	if (over(x, top, w, h - 34) && mouse.wheel)
		fileScroll += Math.sign(mouse.wheel);
	fileScroll = clamp(fileScroll, 0, Math.max(0, files.length - visible));

	if (!files.length) {
		txtBox('No audio files. Upload .wav, .aif or .flac files to the project, ' +
			'or to a samples/ folder in it, in the Bela IDE, then press rescan.',
			x + 10, top + 4, w - 20, 120, 11, C.dim, 'left', 'top');
		return;
	}
	const loaded = board ? Math.round(board[ST.file]) : -1;
	const loading = board && board[ST.loader] === 1;
	const maxChars = Math.floor((w - 24) / (11 * charW));
	for (let i = 0; i < visible && i + fileScroll < files.length; ++i) {
		const idx = i + fileScroll;
		const ry = top + i * rowH;
		const hover = over(x + 4, ry, w - 8, rowH);
		if (idx === loaded)
			fillR(x + 4, ry, w - 8, rowH, 3, C.accent);
		else if (hover)
			fillR(x + 4, ry, w - 8, rowH, 3, C.rowHover);
		let name = files[idx];
		if (name.length > maxChars)
			name = '…' + name.slice(name.length - maxChars + 1);
		txt(name, x + 10, ry + rowH / 2, 11, idx === loaded ? C.ink : C.text, 'left', 'middle');
		if (hover && (mouse.touch ? mouse.tap : mouse.pressed) && !loading)
			send('/load', idx);
	}
	if (files.length > visible)
		txt((fileScroll + 1) + '–' + Math.min(files.length, fileScroll + visible) + ' of ' + files.length, x + w - 8, y + h - 4, 10, C.dim, 'right', 'bottom');
}

// --- primitives -------------------------------------------------------------

function button(x, y, w, h, label, on) {
	const hover = over(x, y, w, h);
	fillR(x, y, w, h, 4, on ? C.accent : hover ? C.btnHover : C.btn);
	strokeC(on ? C.accent : C.btnEdge, 1, 'round');
	ctx.beginPath();
	if (ctx.roundRect) ctx.roundRect(x, y, w, h, 4);
	else ctx.rect(x, y, w, h);
	ctx.stroke();
	txt(label, x + w / 2, y + h / 2, 12, on ? C.ink : C.text, 'center', 'middle');
	return hover && mouse.pressed;
}

function segment(x, y, w, h, label, on, disabled) {
	const hover = !disabled && over(x, y, w, h);
	fillR(x, y, w, h, 3, on && !disabled ? C.accent : hover ? C.btnHover : C.seg);
	txt(label, x + w / 2, y + h / 2, 11, disabled ? C.line : on ? C.ink : C.text, 'center', 'middle');
	return hover && mouse.pressed;
}
