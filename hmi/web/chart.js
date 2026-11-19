// @ts-check
/** Desenho em canvas, sem dependências: série temporal e espectro. */

const COLORS = ['#4ea1ff', '#f2b84b', '#3ccf7a', '#ff7ab6', '#b58cff', '#7fd6ff'];

/** Ajusta o canvas à densidade de pixels e devolve o contexto e o tamanho em CSS px. */
function prepare(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const width = canvas.clientWidth || 400;
  const height = Number(canvas.getAttribute('height')) || 140;
  if (canvas.width !== Math.round(width * ratio)) {
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
    canvas.style.height = `${height}px`;
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);
  return { ctx, width, height };
}

function niceRange(min, max) {
  if (!Number.isFinite(min) || !Number.isFinite(max)) return [0, 1];
  if (min === max) {
    const pad = Math.abs(min) * 0.01 || 0.001;
    return [min - pad, max + pad];
  }
  const pad = (max - min) * 0.08;
  return [min - pad, max + pad];
}

function fmt(v) {
  const a = Math.abs(v);
  if (a !== 0 && (a < 0.01 || a >= 10000)) return v.toExponential(2);
  return v.toFixed(a < 1 ? 4 : a < 100 ? 2 : 0);
}

/**
 * Série temporal com eixo de tempo relativo ao fim (segundos).
 * @param {HTMLCanvasElement} canvas
 * @param {{t:number[], v:number[]}} series
 * @param {{color?:string, frozen?:number[], window:number}} opt
 */
export function drawSeries(canvas, series, opt) {
  const { ctx, width, height } = prepare(canvas);
  const left = 64;
  const right = 8;
  const top = 8;
  const bottom = 18;
  const w = width - left - right;
  const h = height - top - bottom;
  ctx.font = '11px system-ui';
  ctx.fillStyle = '#8b94a3';
  if (!series || series.t.length < 2) {
    ctx.fillText('sem dados', left + 8, top + 16);
    return;
  }
  const tEnd = series.t[series.t.length - 1];
  const tStart = tEnd - opt.window;
  let min = Infinity;
  let max = -Infinity;
  for (const v of series.v) {
    if (v < min) min = v;
    if (v > max) max = v;
  }
  for (const f of opt.frozen || []) {
    if (f < min) min = f;
    if (f > max) max = f;
  }
  [min, max] = niceRange(min, max);
  const x = (t) => left + ((t - tStart) / opt.window) * w;
  const y = (v) => top + (1 - (v - min) / (max - min)) * h;

  ctx.strokeStyle = '#232933';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const gy = top + (i * h) / 4;
    ctx.beginPath();
    ctx.moveTo(left, gy);
    ctx.lineTo(left + w, gy);
    ctx.stroke();
    ctx.fillText(fmt(max - ((max - min) * i) / 4), 2, gy + 4);
  }
  for (let s = 0; s <= opt.window; s += opt.window <= 10 ? 1 : 5) {
    const gx = x(tEnd - s);
    ctx.fillText(`-${s}s`, gx - 10, height - 4);
  }
  for (const f of opt.frozen || []) {
    ctx.strokeStyle = '#7fd6ff';
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(left, y(f));
    ctx.lineTo(left + w, y(f));
    ctx.stroke();
    ctx.setLineDash([]);
  }
  ctx.strokeStyle = opt.color || COLORS[0];
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  let started = false;
  for (let i = 0; i < series.t.length; i++) {
    if (series.t[i] < tStart) continue;
    const px = x(series.t[i]);
    const py = y(series.v[i]);
    if (!started) {
      ctx.moveTo(px, py);
      started = true;
    } else {
      ctx.lineTo(px, py);
    }
  }
  ctx.stroke();
}

/**
 * Espectro de amplitude com marcadores de rotação medida.
 * @param {HTMLCanvasElement} canvas
 * @param {{freq_hz:number[], amplitude:number[], markers:{motor:number,hz:number}[], unit:string}} spec
 * @param {{log:boolean}} opt
 */
export function drawSpectrum(canvas, spec, opt) {
  const { ctx, width, height } = prepare(canvas);
  const left = 70;
  const right = 10;
  const top = 10;
  const bottom = 26;
  const w = width - left - right;
  const h = height - top - bottom;
  ctx.font = '11px system-ui';
  ctx.fillStyle = '#8b94a3';
  if (!spec || spec.freq_hz.length < 2) {
    ctx.fillText('dados insuficientes (precisa de pelo menos 16 amostras)', left + 8, top + 20);
    return;
  }
  const fMax = spec.freq_hz[spec.freq_hz.length - 1];
  const amps = spec.amplitude.slice(1);
  let aMax = Math.max(...amps, 1e-12);
  let aMin = opt.log ? Math.max(Math.min(...amps.filter((a) => a > 0)), aMax * 1e-5) : 0;
  const tr = (a) => (opt.log ? Math.log10(Math.max(a, aMin)) : a);
  const lo = tr(aMin);
  const hi = tr(aMax) + (opt.log ? 0.1 : aMax * 0.05);
  const x = (f) => left + (f / fMax) * w;
  const y = (a) => top + (1 - (tr(a) - lo) / (hi - lo)) * h;

  ctx.strokeStyle = '#232933';
  for (let i = 0; i <= 4; i++) {
    const gy = top + (i * h) / 4;
    ctx.beginPath();
    ctx.moveTo(left, gy);
    ctx.lineTo(left + w, gy);
    ctx.stroke();
    const value = hi - ((hi - lo) * i) / 4;
    ctx.fillText(opt.log ? `1e${value.toFixed(1)}` : fmt(value), 2, gy + 4);
  }
  const step = fMax > 150 ? 25 : fMax > 60 ? 10 : 5;
  for (let f = 0; f <= fMax; f += step) {
    ctx.fillText(`${f}`, x(f) - 6, height - 8);
  }
  ctx.fillText('Hz', left + w - 16, height - 8);

  const colors = ['#ff7ab6', '#f2b84b', '#3ccf7a', '#b58cff'];
  for (const m of spec.markers || []) {
    for (const [k, dash] of [
      [1, []],
      [2, [3, 4]],
    ]) {
      const f = m.hz * k;
      if (f > fMax) continue;
      ctx.strokeStyle = colors[(m.motor - 1) % 4];
      ctx.setLineDash(dash);
      ctx.beginPath();
      ctx.moveTo(x(f), top);
      ctx.lineTo(x(f), top + h);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = colors[(m.motor - 1) % 4];
      ctx.fillText(`M${m.motor}${k === 2 ? '×2' : ''}`, x(f) + 3, top + 12 + (m.motor - 1) * 12);
    }
  }
  ctx.fillStyle = '#8b94a3';
  ctx.strokeStyle = '#4ea1ff';
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  for (let i = 1; i < spec.freq_hz.length; i++) {
    const px = x(spec.freq_hz[i]);
    const py = y(spec.amplitude[i]);
    if (i === 1) ctx.moveTo(px, py);
    else ctx.lineTo(px, py);
  }
  ctx.stroke();
}
