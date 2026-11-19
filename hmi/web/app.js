// @ts-check
/**
 * IHM de bancada: painel ao vivo, congelamento por valor, motores, gráficos,
 * FFT, sessões e calibração. O STM32 decide a segurança; esta página só pede.
 *
 * Nada aqui usa confirm()/prompt(): diálogos bloqueantes param os timers e o
 * heartbeat, o que desarma os motores. Confirmações são por segundo clique.
 */
import { drawSeries, drawSpectrum } from './chart.js';

/** @param {string} id */
const $ = (id) => /** @type {any} */ (document.getElementById(id));
const STATUS_ARMED = 0x01;
const STATUS_SWEEPING = 0x02;
const STATUS_HB = 0x04;
const STATUS_RX_BLOCKED = 0x08;
const STATUS_ESC = 0x10;
const STATUS_POWER_LOW = 0x40;
const SAFE_CAP = 40;

const clientId = sessionStorage.getItem('bench-client') || crypto.randomUUID();
sessionStorage.setItem('bench-client', clientId);

/** @type {{name:string, group:string, label:string, unit:string, digits:number}[]} */
let signals = [];
/** @type {Record<string, any>} */
let byName = {};
/** @type {any} */
let snap = null;
/** @type {any[]} */
let freezes = [];
/** @type {any} */
let config = null;
let isController = false;
const charted = new Set(JSON.parse(localStorage.getItem('bench-charts') || '[]'));
const selected = new Set();
/** @type {Record<string, any>} */
const rows = {};
/** @type {Record<string, any>} */
const groupEls = {};
/** @type {any[]} */
let freezeCells = [];
const desiredReversed = [false, false, false, false];
let directionLoaded = false;
let lastLogKey = '';

// ---------------------------------------------------------------- utilidades

async function api(path, body) {
  const options =
    body === undefined
      ? {}
      : {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ client_id: clientId, ...body }),
        };
  const response = await fetch(path, options);
  const data = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(data.error || response.statusText);
  return data;
}

let bannerTimer = 0;
function toast(text, bad = false) {
  const el = $('banner');
  el.textContent = text;
  el.className = bad ? 'banner bad' : 'banner';
  el.hidden = false;
  clearTimeout(bannerTimer);
  bannerTimer = window.setTimeout(() => (el.hidden = true), 5000);
}

async function command(name, args = {}) {
  try {
    const r = await api('/api/command', { name, args });
    if (!r.ok) toast(`${name}: ${r.result}`, true);
    return r;
  } catch (e) {
    toast(`${name}: ${e.message}`, true);
    return null;
  }
}

/** Botão perigoso: o primeiro clique arma a confirmação por 3 s. */
function confirmClick(button, label, action) {
  const original = button.textContent;
  button.addEventListener('click', () => {
    if (button.dataset.armed === '1') {
      button.dataset.armed = '';
      button.textContent = original;
      action();
      return;
    }
    button.dataset.armed = '1';
    button.textContent = label;
    setTimeout(() => {
      button.dataset.armed = '';
      button.textContent = original;
    }, 3000);
  });
}

function fmt(value, digits) {
  if (value === null || value === undefined || !Number.isFinite(value)) return '—';
  if (Math.abs(value) >= 1e6) return value.toExponential(3);
  return value.toFixed(digits);
}

function fmtSigma(std, digits) {
  if (std === null || std === undefined) return '';
  return `σ ${std === 0 ? '0' : std < 10 ** -(digits + 1) ? std.toExponential(1) : std.toFixed(digits + 1)}`;
}

function el(tag, attrs = {}, ...children) {
  const node = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'class') node.className = v;
    else if (k.startsWith('on')) node.addEventListener(k.slice(2), v);
    else if (v !== undefined && v !== null && v !== false) node.setAttribute(k, v === true ? '' : v);
  }
  for (const c of children) node.append(c instanceof Node ? c : document.createTextNode(String(c)));
  return node;
}

const status = () => (snap && snap.status) || null;
const armed = () => Boolean(status() && status().flags & STATUS_ARMED);
const value = (name) => snap && snap.values[name];

// ---------------------------------------------------------------- abas

for (const b of document.querySelectorAll('.tabs button')) {
  b.addEventListener('click', () => {
    for (const x of document.querySelectorAll('.tabs button')) x.classList.toggle('active', x === b);
    for (const t of document.querySelectorAll('.tab')) t.hidden = t.id !== `tab-${b.dataset.tab}`;
    if (b.dataset.tab === 'session') loadSessions();
  });
}

// ---------------------------------------------------------------- painel ao vivo

function buildGroups() {
  const container = $('groups');
  container.innerHTML = '';
  const groups = [...new Set(signals.map((s) => s.group))];
  for (const group of groups) {
    const box = el(
      'div',
      { class: 'group' },
      el(
        'div',
        { class: 'group-head' },
        el('h2', {}, group),
        el('button', { title: 'Congelar o grupo', onclick: () => freeze({ group }) }, '❄ grupo')
      )
    );
    groupEls[group] = box;
    for (const s of signals.filter((x) => x.group === group)) {
      const dot = el('span', { class: 'dot' });
      const val = el('span', { class: 'vval', title: 'Clique para congelar este valor' }, '—');
      const sig = el('span', { class: 'vsig' });
      const chartBtn = el('button', { title: 'Gráfico' }, '📈');
      const row = el('div', { class: 'vrow' }, dot, el('span', { class: 'vlabel', title: s.name }, s.label), val, sig, chartBtn);
      val.addEventListener('click', () => {
        val.classList.remove('flash');
        void val.offsetWidth;
        val.classList.add('flash');
        freeze({ names: [s.name] });
      });
      chartBtn.addEventListener('click', () => toggleChart(s.name));
      chartBtn.classList.toggle('on', charted.has(s.name));
      rows[s.name] = { row, dot, val, sig, chartBtn, signal: s };
      box.append(row);
    }
    container.append(box);
  }
}

function renderValues() {
  const showAll = $('show-all').checked;
  const visible = {};
  for (const [name, r] of Object.entries(rows)) {
    const v = value(name);
    const s = r.signal;
    const has = Boolean(v);
    r.row.hidden = !has && !showAll;
    if (has) {
      r.val.innerHTML = '';
      r.val.append(fmt(v.v, s.digits), el('span', { class: 'unit' }, s.unit));
      r.dot.className = `dot ${v.ok ? 'ok' : 'stale'}`;
      r.dot.title = v.ok ? 'válido e recente' : `inválido ou velho (${v.age} s)`;
      r.sig.textContent = fmtSigma(v.std, s.digits);
    } else {
      r.val.textContent = '—';
      r.dot.className = 'dot';
      r.sig.textContent = '';
    }
    visible[s.group] = (visible[s.group] || false) || !r.row.hidden;
  }
  for (const [group, box] of Object.entries(groupEls)) box.hidden = !visible[group];
}

async function freeze(target) {
  try {
    const label = $('freeze-label').value.trim();
    await api('/api/freezes', { ...target, label });
    await loadFreezes();
  } catch (e) {
    toast(`congelar: ${e.message}`, true);
  }
}

$('btn-freeze-all').addEventListener('click', () => freeze({}));
$('show-all').addEventListener('change', renderValues);

// ---------------------------------------------------------------- congelados

async function loadFreezes() {
  freezes = await api('/api/freezes');
  for (const id of [...selected]) if (!freezes.some((f) => f.id === id)) selected.delete(id);
  buildFreezes();
}

function buildFreezes() {
  const box = $('frozen');
  box.innerHTML = '';
  freezeCells = [];
  if (!freezes.length) {
    box.append(el('p', { class: 'small' }, 'Nenhuma amostra. Clique num valor do painel (ou ❄) para congelar.'));
  }
  for (const f of [...freezes].reverse()) {
    const label = el('input', { class: 'flabel', value: f.label });
    label.addEventListener('change', () => api(`/api/freezes/${f.id}`, { label: label.value }).then(loadFreezes));
    const note = el('input', { placeholder: 'nota', value: f.note || '' });
    note.addEventListener('change', () => api(`/api/freezes/${f.id}`, { note: note.value }));
    const pick = el('input', { type: 'checkbox', title: 'Selecionar para comparar' });
    pick.checked = selected.has(f.id);
    pick.addEventListener('change', () => {
      if (pick.checked) selected.add(f.id);
      else selected.delete(f.id);
      $('btn-compare').disabled = selected.size !== 2;
    });
    const del = el('button', { class: 'ghost' }, 'apagar');
    confirmClick(del, 'confirmar?', () => api(`/api/freezes/${f.id}`, { delete: true }).then(loadFreezes));
    const when = new Date(f.created_utc).toLocaleTimeString();
    const head = el(
      'div',
      { class: 'freeze-head' },
      pick,
      label,
      note,
      el('span', { class: 'small' }, `${when} · ${f.source} · t=${f.device_t_ms ?? '—'} ms`),
      del
    );
    const table = el(
      'table',
      { class: 'table' },
      el(
        'tr',
        {},
        el('th', {}, 'sinal'),
        el('th', {}, 'congelado'),
        el('th', {}, 'média 1 s ± σ (n)'),
        el('th', {}, 'ao vivo'),
        el('th', {}, 'Δ'),
        el('th', {}, '|Δ|/σ')
      )
    );
    for (const [name, item] of Object.entries(f.values)) {
      const digits = byName[name] ? byName[name].digits : 3;
      const live = el('td', { class: 'num' }, '—');
      const delta = el('td', { class: 'num delta' }, '—');
      const ratio = el('td', { class: 'num' }, '—');
      const tr = el(
        'tr',
        {},
        el('td', { title: name }, `${item.label} [${item.unit}]${item.valid ? '' : ' (inválido)'}`),
        el('td', { class: 'num' }, fmt(item.value, digits)),
        el('td', { class: 'num' }, item.mean_1s === null ? '—' : `${fmt(item.mean_1s, digits)} ± ${fmt(item.std_1s, digits + 1)} (${item.n_1s})`),
        live,
        delta,
        ratio
      );
      table.append(tr);
      freezeCells.push({ name, item, digits, live, delta, ratio, tr });
    }
    box.append(el('div', { class: 'freeze' }, head, table));
  }
  $('btn-compare').disabled = selected.size !== 2;
}

function renderFreezeLive() {
  for (const c of freezeCells) {
    const v = value(c.name);
    if (!v) {
      c.live.textContent = c.delta.textContent = c.ratio.textContent = '—';
      c.tr.classList.remove('flag');
      continue;
    }
    const d = v.v - c.item.value;
    c.live.textContent = fmt(v.v, c.digits);
    c.delta.textContent = `${d >= 0 ? '+' : ''}${fmt(d, c.digits)}`;
    const sigma = c.item.std_1s;
    const r = sigma > 0 ? Math.abs(d) / sigma : null;
    c.ratio.textContent = r === null ? '—' : r.toFixed(1);
    c.tr.classList.toggle('flag', r !== null && r > 3);
  }
}

$('btn-compare').addEventListener('click', () => {
  const [a, b] = freezes.filter((f) => selected.has(f.id));
  const box = $('compare');
  box.innerHTML = '';
  if (!a || !b) return;
  const table = el(
    'table',
    { class: 'table' },
    el('tr', {}, el('th', {}, 'sinal'), el('th', {}, a.label), el('th', {}, b.label), el('th', {}, 'B − A'), el('th', {}, '|B−A|/√(σA²+σB²)'))
  );
  for (const [name, ia] of Object.entries(a.values)) {
    const ib = b.values[name];
    if (!ib) continue;
    const digits = byName[name] ? byName[name].digits : 3;
    const d = ib.value - ia.value;
    const s = Math.sqrt((ia.std_1s || 0) ** 2 + (ib.std_1s || 0) ** 2);
    const r = s > 0 ? Math.abs(d) / s : null;
    const tr = el(
      'tr',
      {},
      el('td', {}, `${ia.label} [${ia.unit}]`),
      el('td', { class: 'num' }, `${fmt(ia.value, digits)} ± ${fmt(ia.std_1s, digits + 1)}`),
      el('td', { class: 'num' }, `${fmt(ib.value, digits)} ± ${fmt(ib.std_1s, digits + 1)}`),
      el('td', { class: 'num delta' }, fmt(d, digits)),
      el('td', { class: 'num' }, r === null ? '—' : r.toFixed(1))
    );
    tr.classList.toggle('flag', r !== null && r > 3);
    table.append(tr);
  }
  box.append(
    el('div', { class: 'compare-box' }, el('div', { class: 'card-head' }, el('h2', {}, `A = ${a.label} · B = ${b.label}`), el('button', { class: 'ghost', onclick: () => (box.innerHTML = '') }, 'fechar')), table)
  );
});

confirmClick($('btn-freeze-clear'), 'confirmar: apagar todas?', () => api('/api/freezes-clear', {}).then(loadFreezes));

// ---------------------------------------------------------------- motores

const motorEls = [];
const sendTimers = {};

function buildMotors() {
  const box = $('motors');
  for (let i = 1; i <= 4; i++) {
    const slider = el('input', { type: 'range', min: 0, max: SAFE_CAP, step: 0.5, value: 0 });
    const out = el('div', { class: 'm-val' }, '0.0 %');
    const sub = el('div', { class: 'm-sub' });
    const pick = el('input', { type: 'checkbox', title: 'Incluir no grupo' });
    pick.checked = true;
    const card = el('div', { class: 'motor' }, el('h3', {}, el('label', { class: 'check' }, pick, `M${i}`)), slider, out, sub);
    slider.addEventListener('input', () => {
      out.textContent = `${Number(slider.value).toFixed(1)} %`;
      slider.dataset.dragging = '1';
      queueMotor({ [i]: Number(slider.value) }, `m${i}`);
    });
    slider.addEventListener('change', () => {
      slider.dataset.dragging = '';
      queueMotor({ [i]: Number(slider.value) }, `m${i}`, true);
    });
    motorEls.push({ slider, out, sub, pick, card });
    box.append(card);
  }
  const toggles = $('dir-toggles');
  for (let i = 0; i < 4; i++) {
    const b = el('button', {}, `M${i + 1}: N`);
    b.addEventListener('click', () => {
      desiredReversed[i] = !desiredReversed[i];
      renderDirectionButtons();
    });
    toggles.append(b);
  }
}

function renderDirectionButtons() {
  const buttons = $('dir-toggles').querySelectorAll('button');
  buttons.forEach((b, i) => {
    b.textContent = `M${i + 1}: ${desiredReversed[i] ? 'invertido' : 'normal'}`;
    b.classList.toggle('warn', desiredReversed[i]);
  });
}

/** Envia no máximo a cada 60 ms por controle; o último valor sempre sai. */
function queueMotor(values, key, now = false) {
  if (!armed()) return;
  clearTimeout(sendTimers[key]);
  const send = () => command('motor_set', { values });
  if (now) send();
  else sendTimers[key] = setTimeout(send, 60);
}

function pickedMotors() {
  return motorEls.map((m, i) => (m.pick.checked ? i + 1 : 0)).filter(Boolean);
}

$('group-slider').addEventListener('input', () => {
  const pct = Number($('group-slider').value);
  $('group-out').textContent = `${pct.toFixed(1)} %`;
  const values = {};
  for (const m of pickedMotors()) values[m] = pct;
  if (Object.keys(values).length) queueMotor(values, 'group');
});

$('btn-arm').addEventListener('click', async () => {
  if (!$('safe-ack').checked) {
    toast('Confirme a condição de segurança (sem hélices ou suporte fixo) antes de armar.', true);
    return;
  }
  await command('arm');
});
$('btn-disarm').addEventListener('click', () => command('stop'));
$('btn-zero').addEventListener('click', () => command('motor_set', { values: { 1: 0, 2: 0, 3: 0, 4: 0 } }));

const capSlider = $('cap');
capSlider.addEventListener('input', () => {
  capSlider.dataset.dragging = '1';
  $('cap-out').textContent = `${capSlider.value} %`;
});
capSlider.addEventListener('change', async () => {
  capSlider.dataset.dragging = '';
  await command('cap', { percent: Number(capSlider.value) });
});
const capUnlock = el('label', { class: 'check danger' }, el('input', { type: 'checkbox', id: 'cap-unlock' }), 'permitir limite acima de 40 %');
capSlider.parentElement.after(capUnlock);
$('cap-unlock').addEventListener('change', () => {
  capSlider.max = $('cap-unlock').checked ? 100 : SAFE_CAP;
});
capSlider.max = SAFE_CAP;

$('rate').addEventListener('input', () => ($('rate-out').textContent = `${$('rate').value} %/s`));
$('rate').addEventListener('change', () => command('rate', { percent_per_s: Number($('rate').value) }));

function directionArgs(save) {
  const reversed = desiredReversed.map((r, i) => (r ? i + 1 : 0)).filter(Boolean);
  return { motors: [1, 2, 3, 4], reversed, save };
}
$('btn-dir').addEventListener('click', () => command('direction', directionArgs(false)));
confirmClick($('btn-dir-save'), 'confirmar gravação no ESC', () => command('direction', directionArgs(true)));
$('btn-beep').addEventListener('click', () => {
  const motors = pickedMotors();
  if (motors.length) command('beep', { motors });
});
$('btn-sweep').addEventListener('click', () => {
  const motors = pickedMotors();
  if (!motors.length) return toast('Selecione ao menos um motor (☑).', true);
  command('sweep', {
    motors,
    max_percent: Number($('sw-max').value),
    step_percent: Number($('sw-step').value),
    hold_s: Number($('sw-hold').value),
  });
});
$('btn-esc').addEventListener('click', async () => {
  try {
    config = await api('/api/config', { motor_poles: Number($('poles').value) });
  } catch (e) {
    toast(e.message, true);
  }
  if (snap && snap.source.live) {
    command('esc_telem', { enabled: $('esc-enable').checked, period_ms: Number($('esc-period').value) });
  } else {
    api('/api/config', { esc_telem: { enabled: $('esc-enable').checked, period_ms: Number($('esc-period').value) } });
  }
});

function renderMotors() {
  const st = status();
  const isArmed = armed();
  const badge = $('arm-badge');
  badge.textContent = isArmed ? (st.flags & STATUS_SWEEPING ? 'ARMADO · degraus' : 'ARMADO') : 'desarmado';
  badge.className = `badge ${isArmed ? 'armed' : ''}`;
  const hb = $('hb-badge');
  const hbOk = st && st.flags & STATUS_HB;
  hb.textContent = st ? (hbOk ? `heartbeat ${st.heartbeat_age_ms} ms` : 'sem heartbeat') : 'heartbeat —';
  hb.className = `badge ${hbOk ? 'ok' : 'muted'}`;
  const canCommand = Boolean(isController && snap && snap.source.live);
  $('btn-arm').disabled = !canCommand || isArmed;
  for (const id of ['btn-dir', 'btn-dir-save', 'btn-beep']) $(id).disabled = !canCommand || isArmed;
  for (const id of ['btn-sweep', 'btn-zero', 'group-slider']) $(id).disabled = !canCommand || !isArmed;
  for (const id of ['cap', 'rate', 'btn-esc']) $(id).disabled = !canCommand;

  const cap = st ? st.cap_permille / 10 : SAFE_CAP;
  if (capSlider.dataset.dragging !== '1' && st) {
    if (cap > SAFE_CAP && !$('cap-unlock').checked) {
      $('cap-unlock').checked = true;
      capSlider.max = 100;
    }
    capSlider.value = String(cap);
    $('cap-out').textContent = `${cap} %`;
  }
  if (st && document.activeElement !== $('rate')) {
    $('rate').value = String(st.rate_permille_s / 10);
    $('rate-out').textContent = `${st.rate_permille_s / 10} %/s`;
  }
  $('group-slider').max = String(cap);
  motorEls.forEach((m, i) => {
    const n = i + 1;
    m.slider.max = String(cap);
    m.slider.disabled = !canCommand || !isArmed;
    if (m.slider.dataset.dragging !== '1') {
      const target = st ? st.target_permille[i] / 10 : 0;
      m.slider.value = String(isArmed ? target : 0);
      m.out.textContent = `${Number(m.slider.value).toFixed(1)} %`;
    }
    const cmd = value(`motor.m${n}.cmd`);
    const dshot = value(`motor.m${n}.dshot`);
    const rpm = value(`esc.m${n}.rpm`);
    m.sub.innerHTML = '';
    m.sub.append(
      `enviado ${fmt(cmd && cmd.v, 1)} %`,
      el('br'),
      `DShot ${fmt(dshot && dshot.v, 0)}`,
      el('br'),
      `RPM ${rpm && rpm.ok ? fmt(rpm.v, 0) : '— (sem KISS)'}`
    );
    m.card.classList.toggle('reversed', Boolean(st && st.reversed & (1 << i)));
  });
  if (st && !directionLoaded) {
    for (let i = 0; i < 4; i++) desiredReversed[i] = Boolean(st.reversed & (1 << i));
    directionLoaded = true;
    renderDirectionButtons();
  }
  if (st) {
    const list = (mask) => [1, 2, 3, 4].filter((n) => mask & (1 << (n - 1))).map((n) => `M${n}`).join(', ') || 'nenhum';
    $('dir-state').textContent = `Invertidos pedidos: ${list(st.reversed)} · transmitidos por completo: ${list(st.direction_sent)} · confirmação do ESC: indisponível (sem leitura de volta).`;
    $('esc-enable').checked = Boolean(st.flags & STATUS_ESC);
  }
}

// ---------------------------------------------------------------- controle LQR

const CT_STATES = ['desarmado', 'armado · medindo o chão', 'subindo a aceleração', 'subindo', 'HOVER', 'pouso'];
const OWNER_CONTROL = 2;
const CT_FLAGS = [
  [0x08, 'BNO'],
  [0x02, 'distância'],
  [0x04, 'fluxo'],
  [0x01, 'Kalman livre'],
  [0x10, 'ajuste do hover'],
  [0x20, 'motor no limite'],
  [0x40, 'XY no controle'],
  [0x80, 'bateria compensada'],
];
const CT_PARAMS = [
  ['cp-height', 'height_m', 100],
  ['cp-hover', 'hover_percent', 1],
  ['cp-cap', 'cap_percent', 1],
  ['cp-spool', 'spool_percent_per_s', 1],
  ['cp-climb', 'climb_m_s', 100],
  ['cp-land', 'land_m_s', 100],
  ['cp-ki', 'trim_ki', 1],
  ['cp-kixy', 'ki_xy', 1],
  ['cp-kiyaw', 'ki_yaw', 1],
];
let ctParamsLoaded = false;
/** @type {Record<string, any>} */
const ctMeas = {};
/** @type {Record<string, any>} */
const ctEst = {};

function ctState() {
  const st = status();
  if (!st || !(st.flags & STATUS_ARMED) || st.owner !== OWNER_CONTROL || !snap.ctrl) return 0;
  return snap.ctrl.state;
}

function buildControl() {
  const meas = [
    ['φ rolagem (direita para baixo +)', 'ctrl.phi', 'deg'],
    ['θ arfagem (nariz para cima +)', 'ctrl.theta', 'deg'],
    ['ψ guinada (horário +, 0 no armamento)', 'ctrl.psi', 'deg'],
    ['p', 'ctrl.p', 'rad/s'],
    ['q', 'ctrl.q', 'rad/s'],
    ['r', 'ctrl.r', 'rad/s'],
    ['altura (acima do chão do armamento)', 'ctrl.h_meas', 'm'],
    ['u frente (fluxo)', 'ctrl.u_meas', 'm/s'],
    ['v direita (fluxo)', 'ctrl.v_meas', 'm/s'],
  ];
  const t = $('ct-meas');
  for (const [label, name, unit] of meas) {
    const out = el('td', { class: 'num' }, '—');
    ctMeas[name] = out;
    t.append(el('tr', {}, el('td', {}, label), out, el('td', {}, unit)));
  }
  const est = [
    ['altura', 'ctrl.h_est', 'm'],
    ['velocidade vertical (para cima)', 'ctrl.vz_est', 'm/s'],
    ['x frente (deriva: só o fluxo integrado)', 'ctrl.x_est', 'm'],
    ['y direita', 'ctrl.y_est', 'm'],
    ['u frente', 'ctrl.u_est', 'm/s'],
    ['v direita', 'ctrl.v_est', 'm/s'],
  ];
  const e = $('ct-est');
  for (const [label, name, unit] of est) {
    const out = el('td', { class: 'num' }, '—');
    ctEst[name] = out;
    e.append(el('tr', {}, el('td', {}, label), out, el('td', {}, unit)));
  }
  const motors = $('ct-motors');
  // Vista de cima, frente para cima: M4 M2 na frente, M3 M1 atrás.
  for (const n of [4, 2, 3, 1]) {
    const num = el('b', {}, '—');
    const bar = el('i', {});
    motors.append(el('div', { 'data-motor': n }, `M${n} ${{ 4: 'frente esq.', 2: 'frente dir.', 3: 'trás esq.', 1: 'trás dir.' }[n]}: `, num, el('div', { class: 'bar' }, bar)));
    ctMotorEls[n] = { num, bar };
  }
  for (const [id] of CT_PARAMS) $(id).addEventListener('input', () => ($('cp-state').textContent = 'alterado, falta aplicar'));
  for (const id of ['cp-autotrim', 'cp-ekf']) $(id).addEventListener('input', () => ($('cp-state').textContent = 'alterado, falta aplicar'));
  $('cp-apply').addEventListener('click', applyCtParams);
  $('ct-arm').addEventListener('click', () => command('ctrl', { action: 'arm' }));
  $('ct-takeoff').addEventListener('click', () => command('ctrl', { action: 'takeoff' }));
  $('ct-land').addEventListener('click', () => command('ctrl', { action: 'land' }));
  $('ct-disarm').addEventListener('click', stopAll);
  $('ct-stop').addEventListener('click', stopAll);
}
/** @type {Record<number, any>} */
const ctMotorEls = {};

// Ganho gerado de cada coluna em % de aceleração por unidade (Kd ÷ 1,755 rad/s por ‰ ÷ 10).
const CT_GAINS = [
  ['z', 'Altura P', 2.0, '%/m'],
  ['w', 'Altura D', 3.06, '%/(m/s)'],
  ['att', 'Rolagem/arfagem P', 2.9 * (Math.PI / 180), '%/°'],
  ['rate', 'Rolagem/arfagem D', 0.66, '%/(rad/s)'],
  ['yaw', 'Guinada P', 0.63 * (Math.PI / 180), '%/°'],
  ['yaw_rate', 'Guinada D', 0.44, '%/(rad/s)'],
  ['xy', 'Posição XY P (× atitude P)', 0.19, '%/m'],
  ['vel', 'Posição XY D (× atitude P)', 0.39, '%/(m/s)'],
];
/** Grupos cujo efeito também escala com o multiplicador de atitude (lqg_gains_t). */
const GAIN_TIMES_ATT = new Set(['xy', 'vel']);
const GAIN_LIMITS = { z: [0, 20], w: [0, 20], att: [0.25, 5], rate: [0.25, 5], yaw: [0, 10], yaw_rate: [0, 10], xy: [0, 10], vel: [0, 10] };
/** @type {Record<string, any>} */
const cgInputs = {};

function buildGains() {
  const table = $('cg-table');
  table.append(el('tr', {}, el('th', {}, 'grupo'), el('th', {}, '×'), el('th', {}, 'efeito')));
  for (const [key, label, base, unit] of CT_GAINS) {
    const [lo, hi] = GAIN_LIMITS[key];
    const input = el('input', { type: 'number', step: '0.25', min: String(lo), max: String(hi), value: '1' });
    const effect = el('td', { class: 'num' }, '');
    const update = () => {
      refreshGainEffects();
      $('cg-state').textContent = 'alterado, falta aplicar';
    };
    input.addEventListener('input', update);
    cgInputs[key] = { input, effect, base, unit };
    table.append(el('tr', {}, el('td', {}, label), el('td', {}, input), effect));
  }
  $('cg-apply').addEventListener('click', applyGains);
  $('cg-reset').addEventListener('click', () => {
    for (const k of Object.keys(cgInputs)) cgInputs[k].input.value = '1';
    for (const k of Object.keys(cgInputs)) cgInputs[k].input.dispatchEvent(new Event('input'));
  });
}

/** Efeito de cada grupo; XY P/D são relativos à atitude P no firmware. */
function refreshGainEffects() {
  const att = cgInputs.att ? Number(cgInputs.att.input.value) : 1;
  for (const [key, c] of Object.entries(cgInputs)) {
    const k = Number(c.input.value) * (GAIN_TIMES_ATT.has(key) ? att : 1);
    c.effect.textContent = `${fmt(c.base * k, 3)} ${c.unit}`;
  }
}

function fillGains() {
  const g = config.gains || {};
  for (const [key, c] of Object.entries(cgInputs)) c.input.value = String(g[key] ?? 1);
  refreshGainEffects();
  $('cg-state').textContent = '';
}

async function applyGains() {
  const args = {};
  for (const [key, c] of Object.entries(cgInputs)) args[key] = Number(c.input.value);
  const r = await command('ctrl_gains', args);
  if (r && r.ok) {
    config = await api('/api/config');
    $('cg-state').textContent = 'aplicado';
  }
}

function fillCtParams() {
  const c = config.control;
  for (const [id, key, scale] of CT_PARAMS) $(id).value = String(Math.round(c[key] * scale * 1000) / 1000);
  $('cp-autotrim').checked = Boolean(c.autotrim);
  $('cp-ekf').checked = Boolean(c.use_ekf);
  $('cp-state').textContent = '';
  fillGains();
  ctParamsLoaded = true;
}

async function applyCtParams() {
  const args = { autotrim: $('cp-autotrim').checked, use_ekf: $('cp-ekf').checked };
  for (const [id, key, scale] of CT_PARAMS) args[key] = Number($(id).value) / scale;
  const r = await command('ctrl_params', args);
  if (r && r.ok) {
    config = await api('/api/config');
    $('cp-state').textContent = 'aplicado';
  }
}

function renderControl() {
  const st = status();
  const state = ctState();
  const canCommand = Boolean(isController && snap.source.live);
  const isArmed = armed();
  const badge = $('ct-state');
  badge.textContent = CT_STATES[state];
  badge.className = `badge ${state === 4 ? 'hover' : state > 0 ? 'armed' : ''}`;
  const hb = $('ct-hb');
  const hbOk = st && st.flags & STATUS_HB;
  hb.textContent = st ? (hbOk ? `heartbeat ${st.heartbeat_age_ms} ms` : 'sem heartbeat') : 'heartbeat —';
  hb.className = `badge ${hbOk ? 'ok' : 'muted'}`;
  $('ct-arm').disabled = !canCommand || isArmed || !$('ct-ack').checked;
  $('ct-takeoff').disabled = !canCommand || state !== 1;
  $('ct-land').disabled = !canCommand || state < 2 || state > 4;
  $('ct-disarm').disabled = !isArmed;
  $('cp-apply').disabled = !canCommand;
  $('cg-apply').disabled = !canCommand;
  const live = state > 0 ? snap.ctrl : null;
  const v = (n) => snap.values[n];
  const show = (id, name, digits, unit, scale = 1) => {
    const x = v(name);
    $(id).textContent = x && live ? `${fmt(x.v * scale, digits)} ${unit}` : '—';
  };
  show('ct-h-est', 'ctrl.h_est', 3, 'm');
  show('ct-h-ref', 'ctrl.h_ref', 3, 'm');
  show('ct-h-meas', 'ctrl.h_meas', 3, 'm');
  show('ct-base', 'ctrl.base', 1, '%');
  show('ct-trim', 'ctrl.trim', 2, '%');
  for (const [name, out] of Object.entries(ctMeas)) {
    const x = v(name);
    out.textContent = x ? fmt(x.v, 3) : '—';
  }
  for (const [name, out] of Object.entries(ctEst)) {
    const x = v(name);
    out.textContent = x ? fmt(x.v, 3) : '—';
  }
  for (const n of [1, 2, 3, 4]) {
    const x = v(`motor.m${n}.cmd`);
    ctMotorEls[n].num.textContent = x ? `${fmt(x.v, 1)} %` : '—';
    ctMotorEls[n].bar.style.width = `${x ? Math.max(0, Math.min(100, x.v)) : 0}%`;
  }
  const flags = $('ct-flags');
  flags.innerHTML = '';
  if (snap.ctrl) {
    for (const [bit, label] of CT_FLAGS) {
      const on = Boolean(snap.ctrl.flags & bit);
      const bad = !on && (bit === 0x08 || bit === 0x02);
      flags.append(el('span', { class: `badge ${on ? (bit === 0x20 ? 'bad' : 'ok') : bad ? 'bad' : 'muted'}` }, label));
    }
  }
  if (!ctParamsLoaded && config && config.control) fillCtParams();
  const items = snap.texts.slice(-12).reverse();
  const log = $('ct-log');
  const key = items.map((t) => `${t.t_ms}${t.text}`).join('|');
  if (log.dataset.key !== key) {
    log.dataset.key = key;
    log.innerHTML = '';
    for (const t of items) log.append(el('div', { class: `l${t.level}` }, `[${t.t_ms} ms] ${t.text}`));
  }
}

// ---------------------------------------------------------------- gráficos

function toggleChart(name) {
  if (charted.has(name)) charted.delete(name);
  else charted.add(name);
  localStorage.setItem('bench-charts', JSON.stringify([...charted]));
  if (rows[name]) rows[name].chartBtn.classList.toggle('on', charted.has(name));
  buildCharts();
}

const chartEls = {};
function buildCharts() {
  const box = $('charts');
  box.innerHTML = '';
  for (const k of Object.keys(chartEls)) delete chartEls[k];
  $('chart-empty').hidden = charted.size > 0;
  for (const name of charted) {
    const s = byName[name];
    if (!s) continue;
    const canvas = el('canvas', { height: 120 });
    const close = el('button', { class: 'ghost', onclick: () => toggleChart(name) }, '×');
    box.append(el('div', { class: 'chart-head' }, el('span', {}, `${s.label} [${s.unit}] · ${s.group}`), close), canvas);
    chartEls[name] = canvas;
  }
}
$('btn-chart-clear').addEventListener('click', () => {
  for (const n of [...charted]) toggleChart(n);
});

async function updateCharts() {
  const names = Object.keys(chartEls);
  if (!names.length || $('tab-panel').hidden) return;
  const windowS = Number($('chart-window').value);
  try {
    const data = await api(`/api/series?names=${encodeURIComponent(names.join(','))}&seconds=${windowS}`);
    names.forEach((name, i) => {
      const frozen = freezes
        .filter((f) => f.values[name])
        .slice(-3)
        .map((f) => f.values[name].value);
      drawSeries(chartEls[name], data[name], { window: windowS, frozen, color: ['#4ea1ff', '#f2b84b', '#3ccf7a', '#ff7ab6', '#b58cff'][i % 5] });
    });
  } catch {
    /* servidor ocupado: tenta de novo */
  }
}

// ---------------------------------------------------------------- FFT

function buildFftSelect() {
  const sel = $('fft-signal');
  for (const s of signals.filter((x) => /^imu\.(gyro|lacc)/.test(x.name) || x.name === 'mtf.distance')) {
    sel.append(el('option', { value: s.name }, `${s.label} [${s.unit}]`));
  }
}

async function updateFft() {
  if ($('tab-analysis').hidden || !$('fft-run').checked) return;
  try {
    const spec = await api(`/api/spectrum?name=${encodeURIComponent($('fft-signal').value)}&seconds=${$('fft-window').value}`);
    drawSpectrum($('fft-canvas'), spec, { log: $('fft-log').checked });
    $('fft-info').textContent = spec.fs_hz
      ? `fs ≈ ${spec.fs_hz.toFixed(1)} Hz · N = ${spec.n} · resolução ${(spec.fs_hz / spec.n).toFixed(2)} Hz · amplitude em ${spec.unit} · ${spec.markers.length ? spec.markers.map((m) => `M${m.motor} ${m.hz.toFixed(1)} Hz`).join(', ') : 'sem RPM medido'}`
      : 'sem dados suficientes';
  } catch (e) {
    $('fft-info').textContent = e.message;
  }
}

// ---------------------------------------------------------------- topo, log, controle

function renderTop() {
  const src = snap.source;
  const pill = $('source-pill');
  const names = { serial: `Placa ${src.port || ''}`, simulator: 'Simulador', replay: `Replay ${src.session || ''}`, none: 'Desconectado' };
  pill.textContent = names[src.kind] || src.kind;
  pill.className = `pill ${src.live ? 'live' : src.kind === 'replay' ? 'replay' : ''}`;
  $('fw-info').textContent = snap.hello ? `fw ${snap.hello.firmware_id} (${snap.hello.variant}) · protocolo ${snap.hello.major}.${snap.hello.minor}` : 'firmware —';
  const l = snap.link;
  $('link-info').textContent = `pacotes ${l.packets} · inválidos ${l.errors} · perdidos ${l.lost}`;
  isController = snap.control.controller === clientId;
  $('control-box').classList.toggle('mine', isController);
  $('control-state').textContent = isController ? 'você controla' : snap.control.controller ? 'outro cliente controla' : 'sem controle';
  $('btn-claim').hidden = isController;
  const rec = $('btn-rec');
  if (snap.recording) {
    const s = Math.floor(snap.recording.elapsed_s);
    rec.textContent = `■ Parar ${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
    rec.classList.add('on');
    $('rec-path').textContent = snap.recording.path;
  } else {
    rec.textContent = '● Gravar';
    rec.classList.remove('on');
  }
  rec.disabled = !src.live && !snap.recording;
  $('replay-box').hidden = src.kind !== 'replay';
  if (src.kind === 'replay') {
    $('replay-pos').textContent = `${src.position_s} / ${src.duration_s} s ${src.finished ? '(fim)' : src.playing ? '' : '(pausado)'}`;
  }
  const st = status();
  const warnings = [];
  if (src.kind === 'none') warnings.push('Sem fonte: conecte a placa ou o simulador na aba “Fonte e sessões”.');
  if (src.kind === 'replay') warnings.push('Replay: comandos desabilitados.');
  if (src.error) warnings.push(`Porta serial: ${src.error}`);
  if (st && st.flags & STATUS_RX_BLOCKED) warnings.push('Overflow na recepção USB do STM32: reconecte.');
  if (st && st.flags & STATUS_POWER_LOW) warnings.push('Tensão abaixo do mínimo configurado (ARM bloqueado).');
  if (src.live && !isController) warnings.push('Somente leitura: assuma o controle para comandar motores.');
  const box = $('warnings');
  const text = warnings.join(' · ');
  if (box.textContent !== text) box.textContent = text;
  box.hidden = !warnings.length;
}

function renderLog() {
  const items = [
    ...snap.texts.map((t) => ({ when: t.host, cls: `l${t.level}`, text: `[${t.t_ms} ms] ${t.text}` })),
    ...snap.events.map((e) => ({ when: e.utc, cls: 'ev', text: `${e.event} ${JSON.stringify(Object.fromEntries(Object.entries(e).filter(([k]) => !['utc', 'event'].includes(k))))}` })),
  ].sort((a, b) => (a.when < b.when ? 1 : -1));
  const key = items.length ? `${items.length}-${items[0].when}-${items[0].text}` : '';
  if (key === lastLogKey) return;
  lastLogKey = key;
  const box = $('log');
  box.innerHTML = '';
  for (const item of items.slice(0, 60)) box.append(el('div', { class: item.cls }, `${new Date(item.when).toLocaleTimeString()} ${item.text}`));
}

async function claim(force = false) {
  try {
    const r = await api('/api/control', { force });
    isController = r.controller;
    if (!isController) toast('Outro cliente está no controle. Clique de novo para assumir à força (para os motores).');
  } catch (e) {
    toast(e.message, true);
  }
}
let claimArmed = false;
$('btn-claim').addEventListener('click', () => {
  claim(claimArmed);
  claimArmed = true;
  setTimeout(() => (claimArmed = false), 3000);
});

async function stopAll() {
  try {
    await api('/api/command', { name: 'stop', args: {} });
  } catch (e) {
    toast(`STOP: ${e.message}`, true);
  }
}
$('btn-stop').addEventListener('click', stopAll);
$('btn-marker').addEventListener('click', () => api('/api/marker', { label: $('freeze-label').value || 'marcador' }).catch((e) => toast(e.message, true)));

$('btn-rec').addEventListener('click', async () => {
  try {
    if (snap.recording) {
      const r = await api('/api/record/stop', {});
      toast(`Sessão salva em ${r.path}`);
    } else {
      await api('/api/record/start', {
        name: $('rec-name').value,
        notes: $('rec-notes').value,
        conditions: {
          surface: $('rec-surface').value,
          lighting: $('rec-light').value,
          sensor_height_m: $('rec-height').value === '' ? null : Number($('rec-height').value),
          propellers: $('rec-props').value,
          esc: $('rec-esc').value,
        },
      });
    }
  } catch (e) {
    toast(`gravação: ${e.message}`, true);
  }
});
$('btn-notes').addEventListener('click', () => api('/api/notes', { text: $('rec-notes').value }).then(() => toast('Notas salvas.')));

document.addEventListener('keydown', (ev) => {
  const typing = ev.target instanceof HTMLElement && /^(INPUT|TEXTAREA|SELECT)$/.test(ev.target.tagName) && !(ev.target instanceof HTMLInputElement && ['range', 'checkbox'].includes(ev.target.type));
  if (ev.key === 'Escape' || (ev.code === 'Space' && !typing)) {
    ev.preventDefault();
    stopAll();
    return;
  }
  if (typing || ev.ctrlKey || ev.metaKey || ev.altKey) return;
  if (ev.key === 'm' || ev.key === 'M') $('btn-marker').click();
  if (ev.key === 'f' || ev.key === 'F') freeze({});
});

// Espaço ativa o botão focado no keyup: sem isto, STOP seguido de "Armar" ou "Decolar"
// focado dispararia o botão de novo.
document.addEventListener('keyup', (ev) => {
  const typing = ev.target instanceof HTMLElement && /^(INPUT|TEXTAREA|SELECT)$/.test(ev.target.tagName) && !(ev.target instanceof HTMLInputElement && ['range', 'checkbox'].includes(ev.target.type));
  if (ev.code === 'Space' && !typing) ev.preventDefault();
});
for (const button of document.querySelectorAll('.ct-buttons button')) button.addEventListener('click', () => button.blur());

window.addEventListener('pagehide', () => {
  navigator.sendBeacon('/api/control', JSON.stringify({ client_id: clientId, release: true }));
});
document.addEventListener('visibilitychange', () => {
  if (!document.hidden && armed() === false && isController) toast('Aba voltou ao primeiro plano. Se os motores estavam girando, foram parados (heartbeat do navegador).');
});

// ---------------------------------------------------------------- sessões e fonte

async function loadPorts() {
  const ports = await api('/api/ports');
  const sel = $('ports');
  sel.innerHTML = '';
  for (const p of ports) sel.append(el('option', { value: p.device }, `${p.device} — ${p.description}`));
  if (!ports.length) sel.append(el('option', { value: '' }, 'nenhuma porta encontrada'));
}

async function setSource(body) {
  try {
    await api('/api/source', body);
    directionLoaded = false;
    toast('Fonte alterada.');
  } catch (e) {
    toast(e.message, true);
  }
}
$('btn-ports').addEventListener('click', loadPorts);
$('btn-connect').addEventListener('click', () => $('ports').value && setSource({ mode: 'serial', port: $('ports').value }));
$('btn-sim').addEventListener('click', () => setSource({ mode: 'simulator' }));
$('btn-none').addEventListener('click', () => setSource({ mode: 'none' }));
$('btn-play').addEventListener('click', () => api('/api/replay', { playing: !snap.source.playing }));
$('replay-speed').addEventListener('change', () => api('/api/replay', { speed: Number($('replay-speed').value) }));
$('btn-sessions').addEventListener('click', loadSessions);

async function loadSessions() {
  loadPorts().catch(() => {});
  const list = await api('/api/sessions');
  const table = $('sessions');
  table.innerHTML = '';
  table.append(el('tr', {}, el('th', {}, 'sessão'), el('th', {}, 'início (UTC)'), el('th', {}, 'duração'), el('th', {}, 'firmware'), el('th', {}, 'notas'), el('th', {})));
  for (const s of list) {
    table.append(
      el(
        'tr',
        {},
        el('td', {}, s.id),
        el('td', {}, s.started_utc || '—'),
        el('td', { class: 'num' }, s.duration_s ? `${s.duration_s} s` : 'em andamento'),
        el('td', {}, s.firmware_id || '—'),
        el('td', {}, s.notes || ''),
        el('td', {}, el('button', { onclick: () => setSource({ mode: 'replay', session: s.id }) }, 'replay'))
      )
    );
  }
}

// ---------------------------------------------------------------- calibração

function fillConfig() {
  const c = config.power_cal;
  $('cal-vgain').value = c.vbat_gain;
  $('cal-vmin').value = c.vbat_min_v;
  $('cal-again').value = c.current_a_per_v;
  $('cal-aoff').value = c.current_offset_v;
  const e = config.estimator;
  $('est-kx').value = e.flow_gyro_kx;
  $('est-ky').value = e.flow_gyro_ky;
  $('est-tau').value = e.height_tau_s;
  $('est-q').value = e.min_quality;
  $('esc-period').value = config.esc_telem.period_ms;
  $('poles').value = config.motor_poles;
  $('stream-on').checked = config.stream.enabled;
  $('stream-div').value = config.stream.bno_divider;
  $('dir-ref').textContent = `Referência da bancada: M${config.reference_reversed.join(' e M')} invertidos. O pedido vale até desligar o ESC, a menos que seja gravado.`;
}

function calArgs() {
  return {
    vbat_gain: Number($('cal-vgain').value) || 0,
    current_a_per_v: Number($('cal-again').value) || 0,
    current_offset_v: Number($('cal-aoff').value) || 0,
    vbat_min_v: Number($('cal-vmin').value) || 0,
  };
}
$('btn-cal').addEventListener('click', async () => {
  if (snap && snap.source.live) await command('power_cal', calArgs());
  else config = await api('/api/config', { power_cal: calArgs() });
  toast('Calibração aplicada (reenviada a cada nova sessão).');
});
$('btn-cal-vgain').addEventListener('click', () => {
  const pin = value('power.vbat_pin');
  const meas = Number($('cal-vmeas').value);
  if (!pin || !(pin.v > 0.05) || !(meas > 0)) return toast('Precisa de PA1 lido (> 0,05 V) e da tensão medida.', true);
  $('cal-vgain').value = (meas / pin.v).toFixed(4);
});
$('btn-cal-zero').addEventListener('click', () => {
  const pin = value('power.current_pin');
  if (!pin) return toast('PB0 sem leitura.', true);
  $('cal-aoff').value = (pin.mean ?? pin.v).toFixed(4);
});
$('btn-cal-again').addEventListener('click', () => {
  const pin = value('power.current_pin');
  const amps = Number($('cal-ameas').value);
  const offset = Number($('cal-aoff').value) || 0;
  if (!pin || !(amps > 0) || !(pin.v - offset > 0.005)) return toast('Precisa de corrente conhecida > 0 e PB0 acima do offset.', true);
  $('cal-again').value = (amps / (pin.v - offset)).toFixed(3);
});
$('btn-est').addEventListener('click', async () => {
  config = await api('/api/config', {
    estimator: {
      flow_gyro_kx: Number($('est-kx').value) || 0,
      flow_gyro_ky: Number($('est-ky').value) || 0,
      height_tau_s: Number($('est-tau').value) || 0,
      min_quality: Number($('est-q').value) || 0,
    },
  });
  toast('Estimativa atualizada.');
});
$('btn-est-reset').addEventListener('click', () => api('/api/estimator/reset', {}));
$('btn-stream').addEventListener('click', () => command('stream', { enabled: $('stream-on').checked, bno_divider: Number($('stream-div').value) }));

function renderCalLive() {
  const parts = ['power.vbat_pin', 'power.current_pin', 'power.vbat', 'power.current'].map((n) => {
    const v = value(n);
    return `${byName[n].label}: ${v ? fmt(v.v, 3) : '—'} ${byName[n].unit}`;
  });
  $('cal-live').textContent = parts.join(' · ');
}

// ---------------------------------------------------------------- laços

function onSnapshot(data) {
  snap = data;
  renderTop();
  renderValues();
  renderMotors();
  renderControl();
  renderFreezeLive();
  renderLog();
  if (!$('tab-config').hidden) renderCalLive();
}

function connectStream() {
  const source = new EventSource('/api/stream');
  source.onmessage = (ev) => onSnapshot(JSON.parse(ev.data));
  source.onerror = () => {
    toast('Sem conexão com o servidor da IHM; tentando de novo…', true);
  };
}

setInterval(() => {
  if (isController) {
    api('/api/heartbeat', {})
      .then((r) => {
        if (!r.controller) isController = false;
      })
      .catch(() => {});
  }
}, 100);
setInterval(updateCharts, 250);
setInterval(updateFft, 1000);

async function init() {
  signals = await api('/api/signals');
  byName = Object.fromEntries(signals.map((s) => [s.name, s]));
  config = await api('/api/config');
  buildGroups();
  buildMotors();
  buildControl();
  buildGains();
  buildCharts();
  buildFftSelect();
  fillConfig();
  await loadFreezes();
  await claim(false);
  connectStream();
}

init().catch((e) => toast(`Falha ao iniciar: ${e.message}`, true));
