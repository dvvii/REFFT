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
const HOLD_MS = 600;
const PAD = 12;
const C = {
	bg: 22, panel: 32, line: 60, text: 210, dim: 120, accent: '#4fc1b0',
	region: 'rgba(79,193,176,0.16)', wave: '#8fa3b0', head: '#f0c060', warn: '#e06c75',
};

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
function decodeText(b) {
	return new TextDecoder().decode(Uint8Array.from(b, (c) => c.charCodeAt(0)));
}

function fresh(i) {
	const b = Bela.data.buffers[i];
	if (!b || b.count === counts[i])
		return null;
	counts[i] = b.count;
	return b;
}

function readBuffers() {
	if (!Bela.data || !Bela.data.buffers)
		return;
	let b;
	if ((b = fresh(0)) && b.length >= ST.size) {
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
		send('/sync', 1);
	}
}

function connected() {
	return board && performance.now() - statusTs < 1000;
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
}

// End the held pointer, if any: the next frame drops its drag
function release() {
	if (null === ptr.id) return;
	ptr.id = null;
	mouse.released = true;
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
		if (mouse.touch && performance.now() - ptr.prevTapMs < 350 && dist(ptrX, ptrY, ptr.prevTapX, ptr.prevTapY) < 30)
			mouse.double = true;
		ptr.x0 = ptrX; ptr.y0 = ptr.lastY = ptrY; ptr.moved = false; ptr.scroll = 0;
		e.preventDefault();
	});
	el.addEventListener('pointermove', (e) => {
		if (null !== ptr.id && e.pointerId !== ptr.id) return;
		setPtr(e);
		if (null === ptr.id || 'mouse' === e.pointerType) return;
		if (dist(ptrX, ptrY, ptr.x0, ptr.y0) > 10) ptr.moved = true;
		// a swipe that no widget owns scrolls, one list row per 22 px
		if (!drag) {
			ptr.scroll += ptr.lastY - ptrY;
			while (abs(ptr.scroll) >= 22) {
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
	el.addEventListener('pointerleave', (e) => { if ('mouse' === e.pointerType && null === ptr.id) ptrX = ptrY = -1; });
	el.addEventListener('dblclick', () => { if (!mouse.touch) mouse.double = true; });
	el.addEventListener('touchstart', (e) => e.preventDefault(), { passive: false });
	el.addEventListener('touchmove', (e) => e.preventDefault(), { passive: false });
}

function mouseWheel(e) { mouse.wheel += e.delta; return false; }

function over(x, y, w, h) {
	return ptrX >= x && ptrX < x + w && ptrY >= y && ptrY < y + h;
}

function keyPressed() {
	if (' ' === key) setVal('play', '/play', val('play') > 0.5 ? 0 : 1);
	else if ('Enter' === key || 'Home' === key) send('/retrigger', 1);
	else if ('l' === key) setVal('loop', '/loop', val('loop') > 0.5 ? 0 : 1);
	else return true;
	return false;
}

// --- p5 lifecycle -----------------------------------------------------------

let wave = null;     // cached waveform image
let waveDirty = true;
let fileScroll = 0;

function setup() {
	const cnv = createCanvas(windowWidth, windowHeight);
	attachPointer(cnv.elt);
	textFont('monospace');
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
}

// An exception out of draw() stops p5's loop and freezes the page, so one
// bad frame is logged (once per distinct message) and the next one drawn.
let lastError = '';

function draw() {
	try {
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
	if (ptr.clearHover && null === ptr.id) ptrX = ptrY = -1;
	ptr.clearHover = false;
}

function drawFrame() {
	readBuffers();
	background(C.bg);

	const narrow = width < 720;
	let y = drawHeader(PAD);
	const listW = narrow ? width - 2 * PAD : min(300, width * 0.3);
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
		drawFiles(PAD, my + PAD, listW, max(160, height - my - 2 * PAD));
	else
		drawFiles(PAD, y, listW, height - y - PAD);
}

// --- header -----------------------------------------------------------------

function drawHeader(y) {
	const h = 34;
	fill(C.text); noStroke(); textSize(16); textAlign(LEFT, CENTER);
	text('REFFT', PAD, y + h / 2);
	const on = connected();
	fill(on ? C.accent : C.warn);
	circle(PAD + 70, y + h / 2, 9);
	fill(C.dim); textSize(11);
	text(on ? 'connected' : 'waiting for refft-cpp', PAD + 81, y + h / 2);

	fill(C.dim); textAlign(RIGHT, CENTER);
	text('sampler → out L · patch it into in R to stamp it', width - PAD, y + h / 2);
	return y + h + PAD;
}

// --- waveform ---------------------------------------------------------------

// One offscreen image, redrawn in place. p5 keeps every createGraphics() in
// the page until remove(), so making a new one per file or resize leaks a
// canvas each time, and iPad Safari kills the page at its canvas memory cap.
function renderWave(w, h) {
	if (!wave || wave.width !== w || wave.height !== h) {
		if (wave)
			wave.remove();
		wave = createGraphics(w, h);
		wave.pixelDensity(1);
	}
	wave.background(C.panel);
	wave.stroke(C.wave);
	const bins = (peaks.length - 1) / 2;
	for (let x = 0; x < w; ++x) {
		const b0 = floor(x * bins / w);
		const b1 = max(b0 + 1, floor((x + 1) * bins / w));
		let lo = 0, hi = 0;
		for (let b = b0; b < b1; ++b) {
			lo = min(lo, peaks[1 + 2 * b]);
			hi = max(hi, peaks[2 + 2 * b]);
		}
		wave.line(x, h / 2 - hi * h * 0.48, x, h / 2 - lo * h * 0.48);
	}
	waveDirty = false;
}

function fmtTime(s) {
	const m = floor(s / 60);
	const sec = s - 60 * m;
	return m + ':' + (sec < 10 ? '0' : '') + sec.toFixed(1);
}

function drawWaveform(x, y, w, h) {
	if (!board || board[ST.clip] <= 0) {
		fill(C.panel); noStroke(); rect(x, y, w, h, 4);
		fill(C.dim); textSize(12); textAlign(CENTER, CENTER);
		text(board && board[ST.loader] === 1 ? 'opening…' : 'no file loaded: pick one on the left', x + w / 2, y + h / 2);
		return y + h;
	}
	if (peaks && peaksId === board[ST.clip]) {
		if (waveDirty || !wave || wave.width !== w || wave.height !== h)
			renderWave(w, h);
		image(wave, x, y);
	} else {
		// the overview comes from a pass through the whole file (once per file)
		fill(C.panel); noStroke(); rect(x, y, w, h, 4);
		const p = board[ST.analysed];
		fill(C.line); rect(x, y + h - 4, p * w, 4);
		fill(C.dim); textSize(12); textAlign(CENTER, CENTER);
		text('reading the file for its waveform and peak… ' + floor(100 * p) + '%', x + w / 2, y + h / 2);
	}

	let s = val('start'), e = val('end');
	const xs = x + s * w, xe = x + e * w;
	const seconds = board[ST.seconds];

	// interaction: drag an edge, drag out a new region, click to seek,
	// double-click for the whole clip
	const t = constrain((ptrX - x) / w, 0, 1);
	if (mouse.pressed && over(x, y, w, h)) {
		if (abs(ptrX - xs) < 8) drag = { id: 'wave', mode: 'start' };
		else if (abs(ptrX - xe) < 8) drag = { id: 'wave', mode: 'end' };
		else drag = { id: 'wave', mode: 'new', anchor: t, x0: ptrX, moved: false };
	}
	if (drag && drag.id === 'wave') {
		if ('start' === drag.mode) {
			s = min(t, e - 0.001);
			setVal('start', '/start', s);
		} else if ('end' === drag.mode) {
			e = max(t, s + 0.001);
			setVal('end', '/end', e);
		} else {
			if (abs(ptrX - drag.x0) > 4)
				drag.moved = true;
			if (drag.moved) {
				s = min(drag.anchor, t);
				e = max(drag.anchor, t);
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
	noStroke(); fill(0, 120);
	rect(x, y, rs - x, h);
	rect(re, y, x + w - re, h);
	fill(C.region);
	rect(rs, y, re - rs, h);
	stroke(C.accent); strokeWeight(2);
	line(rs, y, rs, y + h);
	line(re, y, re, y + h);
	const hx = x + board[ST.head] * w;
	stroke(C.head); strokeWeight(1.5);
	line(hx, y, hx, y + h);
	noStroke();
	if (over(x, y, w, h) && !drag) {
		cursor(abs(ptrX - rs) < 8 || abs(ptrX - re) < 8 ? 'ew-resize' : 'crosshair');
	} else if (!drag)
		cursor(ARROW);

	fill(C.dim); textSize(10); textAlign(LEFT, TOP);
	text(fmtTime(s * seconds) + ' → ' + fmtTime(e * seconds) + '   (' + fmtTime((e - s) * seconds) + ')', x, y + h + 3);
	textAlign(RIGHT, TOP);
	text(fmtTime(board[ST.head] * seconds) + ' / ' + fmtTime(seconds), x + w, y + h + 3);
	return y + h + 16;
}

// --- transport --------------------------------------------------------------

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
	for (const [key, label] of [['loop', 'loop'], ['normalize', 'normalise']]) {
		const on = val(key) > 0.5;
		const bw = 14 + label.length * 8;
		if (button(cx, y, bw, h, label, on))
			setVal(key, '/' + key, on ? 0 : 1);
		cx += bw + 4;
	}
	cx += 10;
	const mono = board && board[ST.channels] < 2;
	fill(mono ? C.line : C.dim); noStroke(); textSize(11); textAlign(LEFT, CENTER);
	text('channel', cx, y + h / 2);
	cx += 60;
	const ch = Math.round(val('channel'));
	CHANNELS.forEach((label, i) => {
		if (segment(cx + i * 40, y, 38, h, label, i === ch, mono))
			setVal('channel', '/channel', i);
	});
	cx += CHANNELS.length * 40 + 12;

	// file info and loader messages
	fill(board && board[ST.loader] === 2 ? C.warn : C.dim);
	textSize(11); textAlign(LEFT, TOP);
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
		textAlign(LEFT, CENTER);
		text(info, cx, y, x + w - cx, h);
		return y + h;
	}
	text(info, x, y + h + 6, w, 30);
	return y + h + 30;
}

// --- sliders ----------------------------------------------------------------

function drawSliders(x, y, w) {
	const cols = w < 520 ? 1 : 2;
	const sw = (w - (cols - 1) * 2 * PAD) / cols;
	const pos = (i) => [x + (i % cols) * (sw + 2 * PAD), y + floor(i / cols) * 40];

	let [sx, sy] = pos(0);
	slider('gain', sx, sy, sw, 'gain', val('gain'),
		(v) => (v + 36) / 60, (t) => -36 + 60 * t,
		(v) => (v >= 0 ? '+' : '') + v.toFixed(1) + ' dB',
		(v) => setVal('gain', '/gain', v), 0);
	[sx, sy] = pos(1);
	slider('xfade', sx, sy, sw, 'crossfade', val('xfade'),
		(v) => Math.log(v / 2) / Math.log(1000), (t) => 2 * Math.pow(1000, t),
		(v) => v < 1000 ? v.toFixed(0) + ' ms' : (v / 1000).toFixed(2) + ' s',
		(v) => setVal('xfade', '/xfade', v), 50);
	return pos(1)[1] + 34;
}

// Drag to set, shift for fine steps, wheel to nudge, double-click to reset.
function slider(id, x, y, w, label, value, toT, fromT, fmt, onSet, def) {
	const ty = y + 22;
	const t = constrain(toT(value), 0, 1);
	fill(C.dim); noStroke(); textSize(11); textAlign(LEFT, BASELINE);
	text(label, x, y + 11);
	fill(C.text); textAlign(RIGHT, BASELINE);
	text(fmt(value), x + w, y + 11);
	fill(C.line); rect(x, ty - 2, w, 4, 2);
	fill(C.accent); rect(x, ty - 2, t * w, 4, 2);
	fill(230); circle(x + t * w, ty, 12);

	const hit = over(x - 6, y, w + 12, 32);
	if (mouse.pressed && hit)
		drag = { id: id, x0: ptrX, t0: t };
	if (drag && drag.id === id) {
		const nt = keyIsDown(SHIFT)
			? drag.t0 + (ptrX - drag.x0) / w * 0.1
			: (ptrX - x) / w;
		onSet(fromT(constrain(nt, 0, 1)));
	}
	if (hit && mouse.wheel)
		onSet(fromT(constrain(t - Math.sign(mouse.wheel) * 0.01, 0, 1)));
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
		const t = constrain((db + 60) / 60, 0, 1);
		fill(C.dim); noStroke(); textSize(10); textAlign(LEFT, BASELINE);
		let label = METERS[m];
		if (board && 3 === m)
			label += board[ST.limitRDb] < -0.1 ? ' · lim ' + board[ST.limitRDb].toFixed(0) + ' dB' : '';
		text(label, mx, y + 10);
		textAlign(RIGHT, BASELINE);
		text(db > -60 ? db.toFixed(0) + ' dB' : '–', mx + mw, y + 10);
		fill(C.panel); rect(mx, y + 14, mw, 8, 2);
		fill(db > -1 ? C.warn : db > -12 ? C.head : C.accent);
		rect(mx, y + 14, t * mw, 8, 2);
	}
	return y + 24;
}

// --- Pepper panel -------------------------------------------------------------

// Read only: buttons 1-4, then the knobs of both layers. The live layer is
// lit; a knob of it that is waiting for pickup shows where the physical knob
// is (orange tick) against the stored value it has to reach.
function drawPanel(x, y, w) {
	if (!board)
		return y;
	const layer = board[ST.layer];
	const chips = [
		['1 shift', layer > 0.5 ? 'layer B' : 'layer A', layer > 0.5],
		['2 freeze', board[ST.freeze] > 0.5 ? 'frozen' : 'off', board[ST.freeze] > 0.5],
		['3 FFT', board[ST.fft] + ' (' + (1000 * board[ST.fft] * 1.25 / 44100).toFixed(0) + ' ms)', false],
		['4 bypass', board[ST.bypass] > 0.5 ? 'bypassed' : 'off', board[ST.bypass] > 0.5],
	];
	const share = [1, 1, 1.8, 1];
	const unit = (w - 3 * 6) / share.reduce((t, v) => t + v);
	let cx = x;
	chips.forEach(([label, value, on], i) => {
		const cw = share[i] * unit;
		fill(on ? C.accent : C.panel); noStroke(); rect(cx, y, cw, 26, 4);
		fill(on ? 20 : C.dim); textSize(10); textAlign(LEFT, CENTER);
		text(label, cx + 8, y + 13);
		fill(on ? 20 : C.text); textSize(11); textAlign(RIGHT, CENTER);
		text(value, cx + cw - 8, y + 13);
		cx += cw + 6;
	});
	y += 26 + 8;

	const touched = Math.round(board[ST.touched]);
	const kw = w / 8;
	for (let l = 0; l < 2; ++l) {
		const live = Math.round(layer) === l;
		const rowH = 58;
		fill(live ? 40 : 28); noStroke(); rect(x, y, w, rowH, 4);
		for (let k = 0; k < 8; ++k) {
			const def = KNOBS[l][k];
			const cx = x + k * kw + kw / 2;
			const v = board[ST.knobs + 8 * l + k];
			const waiting = live && board[ST.picked + 8 * l + k] < 0.5;
			const hot = live && touched === k && !waiting;
			fill(live ? C.dim : C.line); textSize(9); textAlign(CENTER, TOP);
			text((l ? 'B' : 'A') + k + (def ? ' ' + def[0] : ''), x + k * kw + 2, y + 3, kw - 4, 12);
			if (!def)
				continue;
			const r = min(14, kw / 2 - 6);
			const cy = y + 30;
			const a0 = PI * 0.75, a1 = PI * 2.25;
			noFill(); strokeWeight(3); strokeCap(SQUARE);
			stroke(C.line); arc(cx, cy, 2 * r, 2 * r, a0, a1);
			stroke(hot ? C.head : live ? C.accent : C.wave);
			if (v > 0.002) arc(cx, cy, 2 * r, 2 * r, a0, a0 + v * (a1 - a0));
			if (waiting) {
				const p = a0 + board[ST.pots + k] * (a1 - a0);
				stroke(C.head); strokeWeight(2);
				line(cx + cos(p) * (r - 5), cy + sin(p) * (r - 5), cx + cos(p) * (r + 3), cy + sin(p) * (r + 3));
			}
			noStroke(); strokeCap(ROUND);
			fill(waiting ? C.head : live ? C.text : C.dim); textSize(10); textAlign(CENTER, BASELINE);
			text(waiting ? 'turn to ' + pct(v) : def[1](v), cx, y + rowH - 3);
		}
		y += rowH + 4;
	}
	return y - 4;
}

// The field (in R) per display band: level bars, lit by how much of the band
// counts as events; the event band (B6, B7) is the bright strip.
function drawSpectrum(x, y, w) {
	if (!spectrum || spectrum.length < 2 + 2 * 42)
		return y;
	const n = (spectrum.length - 2) / 2;
	const h = 80, lo = -100, hi = -20;
	const octaves = 7, f0 = 100;
	const xOf = (hz) => x + w * Math.log2(hz / f0) / octaves;
	fill(C.panel); noStroke(); rect(x, y, w, h + 16, 4);
	const bx0 = xOf(max(f0, spectrum[0])), bx1 = xOf(min(f0 * 128, spectrum[1]));
	fill(79, 193, 176, 30); rect(bx0, y, bx1 - bx0, h);
	stroke(C.accent); strokeWeight(1);
	line(bx0, y, bx0, y + h); line(bx1, y, bx1, y + h);
	noStroke();
	const bw = w / n;
	for (let b = 0; b < n; ++b) {
		const db = spectrum[2 + b], ev = constrain(spectrum[2 + n + b], 0, 1);
		const t = constrain((db - lo) / (hi - lo), 0, 1);
		fill(lerpColor(color(C.wave), color(C.head), ev));
		rect(x + b * bw + 1, y + h - t * h, bw - 2, t * h);
	}
	fill(C.dim); textSize(9); textAlign(CENTER, TOP);
	for (const hz of [100, 200, 500, 1000, 2000, 5000, 10000])
		text(fmtHz(hz), xOf(hz), y + h + 3);
	textAlign(LEFT, TOP);
	text('in R spectrum · yellow = events · band ' + fmtHz(spectrum[0]) + '–' + fmtHz(spectrum[1]) + ' (B6, B7)', x + 6, y + 4);
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
	return max(0, timer.accMs + (timer.running ? Date.now() - timer.startMs : 0));
}

function drawTimer(x, y, w) {
	const h = 96;
	fill(C.panel); noStroke(); rect(x, y, w, h, 4);
	const bw = min(130, w * 0.25), bh = (h - 3 * 8) / 2, bx = x + w - bw - 8;
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
	const sec = floor(timerMs() / 1000);
	const pad2 = (v) => String(v).padStart(2, '0');
	const label = pad2(floor(sec / 3600)) + ':' + pad2(floor(sec / 60) % 60) + ':' + pad2(sec % 60);
	const tw = bx - x - 16;
	textSize(100);
	textSize(min(h * 0.8, 100 * tw / textWidth(label)));
	fill(timer.running ? C.text : C.dim); textAlign(CENTER, CENTER);
	text(label, x + 8 + tw / 2, y + h / 2 + 2);
	return y + h;
}

// CV outs 1-8 as 0-5 V bars
function drawCv(x, y, w) {
	if (!board)
		return y;
	const h = 46;
	const cw = w / 8;
	fill(C.panel); noStroke(); rect(x, y, w, h + 18, 4);
	for (let c = 0; c < 8; ++c) {
		const v = constrain(board[ST.cv + c], 0, 1);
		const bx = x + c * cw + 8, bw = cw - 16;
		fill(C.dim); textSize(9); textAlign(CENTER, TOP);
		text('CV' + c + ' ' + CVS[c], x + c * cw + 2, y + 3, cw - 4, 12);
		fill(C.bg); rect(bx, y + 16, bw, h - 16, 2);
		fill(4 === c ? (v > 0.5 ? C.head : C.line) : C.accent);
		rect(bx, y + h - v * (h - 16), bw, v * (h - 16), 2);
		fill(C.text); textSize(10); textAlign(CENTER, BASELINE);
		text((5 * v).toFixed(2) + ' V', x + c * cw + cw / 2, y + h + 14);
	}
	return y + h + 18;
}

// --- files ------------------------------------------------------------------

function drawFiles(x, y, w, h) {
	fill(C.panel); noStroke(); rect(x, y, w, h, 4);
	fill(C.text); textSize(12); textAlign(LEFT, CENTER);
	text('files', x + 10, y + 16);
	if (button(x + w - 80, y + 4, 72, 24, 'rescan', false))
		send('/rescan', 1);

	const rowH = 22;
	const top = y + 34;
	const visible = floor((h - 40) / rowH);
	if (over(x, top, w, h - 34) && mouse.wheel)
		fileScroll += Math.sign(mouse.wheel);
	fileScroll = constrain(fileScroll, 0, max(0, files.length - visible));

	if (!files.length) {
		fill(C.dim); textSize(11); textAlign(LEFT, TOP);
		text('No audio files. Upload .wav, .aif or .flac files to the project, ' +
			'or to a samples/ folder in it, in the Bela IDE, then press rescan.',
			x + 10, top + 4, w - 20, 120);
		return;
	}
	const loaded = board ? Math.round(board[ST.file]) : -1;
	const loading = board && board[ST.loader] === 1;
	for (let i = 0; i < visible && i + fileScroll < files.length; ++i) {
		const idx = i + fileScroll;
		const ry = top + i * rowH;
		const hover = over(x + 4, ry, w - 8, rowH);
		if (idx === loaded) {
			fill(C.accent); rect(x + 4, ry, w - 8, rowH, 3);
		} else if (hover) {
			fill(48); rect(x + 4, ry, w - 8, rowH, 3);
		}
		fill(idx === loaded ? 20 : C.text); textSize(11); textAlign(LEFT, CENTER);
		let name = files[idx];
		const maxChars = floor((w - 24) / 6.6);
		if (name.length > maxChars)
			name = '…' + name.slice(name.length - maxChars + 1);
		text(name, x + 10, ry + rowH / 2);
		if (hover && (mouse.touch ? mouse.tap : mouse.pressed) && !loading)
			send('/load', idx);
	}
	if (files.length > visible) {
		fill(C.dim); textAlign(RIGHT, BOTTOM); textSize(10);
		text((fileScroll + 1) + '–' + min(files.length, fileScroll + visible) + ' of ' + files.length, x + w - 8, y + h - 4);
	}
}

// --- primitives -------------------------------------------------------------

function button(x, y, w, h, label, on) {
	const hover = over(x, y, w, h);
	fill(on ? C.accent : hover ? 58 : 44); stroke(on ? C.accent : 70); strokeWeight(1);
	rect(x, y, w, h, 4);
	noStroke(); fill(on ? 20 : C.text); textSize(12); textAlign(CENTER, CENTER);
	text(label, x + w / 2, y + h / 2);
	return hover && mouse.pressed;
}

function segment(x, y, w, h, label, on, disabled) {
	const hover = !disabled && over(x, y, w, h);
	fill(on && !disabled ? C.accent : hover ? 58 : 40); noStroke();
	rect(x, y, w, h, 3);
	fill(disabled ? C.line : on ? 20 : C.text); textSize(11); textAlign(CENTER, CENTER);
	text(label, x + w / 2, y + h / 2);
	return hover && mouse.pressed;
}
