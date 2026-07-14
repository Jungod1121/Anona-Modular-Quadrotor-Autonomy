const I18N = {
  en: {
    title: 'Sim Control',
    runMode: 'Mode',
    modeSingle: 'Single',
    modeMulti: 'Multi',
    planner: 'Planner',
    plannerHint: 'Pick Path A / B / C, then Start.',
    multiMode: 'Multi mission',
    multiHint: 'EGO-Swarm is the official multi core; shared-field / formation use homemade planners.',
    numDrones: 'Drones',
    formation: 'Formation',
    map: 'Map',
    mapHint: 'Official EGO or homemade maps. See <code>MAPS.md</code>.',
    mapSeed: 'Map seed',
    maxVel: 'Max vel (m/s)',
    useRviz: 'Launch RViz2 with the stack',
    start: 'Start',
    restart: 'Restart',
    stop: 'Stop',
    equivCmd: 'Equivalent command',
    live: 'Live',
    position: 'Position',
    velocity: 'Velocity',
    uptime: 'Uptime',
    pid: 'PID',
    goal: 'Goal',
    goalHint: 'Publish goals in RViz with <strong>2D Goal Pose</strong>. Height uses cruise_height (default 1.0 m).',
    presets: 'Presets',
    presetCross: 'Corridor ±15',
    presetSwarm: 'EGO-Swarm ×2',
    presetHome: 'Homemade defaults',
    processLog: 'Process log',
    clearView: 'Clear view',
    footerPrefix: 'Local only',
    footerSuffix: 'plant stays custom (no SO3)',
    idle: 'Idle',
    running: 'Running',
    offline: 'Offline',
    startFailed: 'Start failed',
    familyOfficial: 'Official',
    familyHomemade: 'Homemade',
    homemadeLabel: 'Path A — Homemade planner',
    homemadeDesc: 'Self-developed drone_planner + drone_map',
    egoLabel: 'Path B — Official EGO',
    egoDesc: 'ego_planner + map_generator + our plant',
    gcopterLabel: 'Path C — GCOPTER / MINCO',
    gcopterDesc: 'Vendored GCOPTER + our plant',
  },
  zh: {
    title: '仿真控制台',
    runMode: '模式',
    modeSingle: '单机',
    modeMulti: '多机',
    planner: '规划器',
    plannerHint: '选择路径 A / B / C，再点启动。',
    multiMode: '多机任务',
    multiHint: 'EGO-Swarm 为官方多机核心；同场 / 编队仍用自研规划器。',
    numDrones: '机数',
    formation: '队形',
    map: '地图',
    mapHint: '官方 EGO 或自研地图。见 <code>MAPS.md</code>。',
    mapSeed: '地图种子',
    maxVel: '最大速度 (m/s)',
    useRviz: '一并启动 RViz2',
    start: '启动',
    restart: '重启',
    stop: '停止',
    equivCmd: '等价命令',
    live: '状态',
    position: '位置',
    velocity: '速度',
    uptime: '运行时长',
    pid: 'PID',
    goal: '目标点',
    goalHint: '请在 RViz 用 <strong>2D Goal Pose</strong> 发布目标。高度为 cruise_height（默认 1.0 m）。',
    presets: '预设',
    presetCross: '走廊 ±15',
    presetSwarm: 'EGO-Swarm ×2',
    presetHome: '自研默认',
    processLog: '进程日志',
    clearView: '清空显示',
    footerPrefix: '仅本地',
    footerSuffix: '植物端保持自研（不用 SO3）',
    idle: '空闲',
    running: '运行中',
    offline: '离线',
    startFailed: '启动失败',
    familyOfficial: '官方',
    familyHomemade: '自研',
    homemadeLabel: '路径 A — 自研规划器',
    homemadeDesc: '自研 drone_planner + drone_map',
    egoLabel: '路径 B — 官方 EGO',
    egoDesc: 'ego_planner + map_generator + 自研植物',
    gcopterLabel: '路径 C — GCOPTER / MINCO',
    gcopterDesc: '移植 GCOPTER + 自研植物',
  },
};

const PLANNER_I18N = {
  homemade: { label: 'homemadeLabel', desc: 'homemadeDesc' },
  ego: { label: 'egoLabel', desc: 'egoDesc' },
  gcopter: { label: 'gcopterLabel', desc: 'gcopterDesc' },
};

const MAP_ORDER = [
  'official_forest', 'official_perlin', 'official_posts',
  'official_maze2d', 'official_maze3d',
  'dense_field', 'sparse', 'narrow_corridor',
  'ego_maze2d_port', 'ego_forest_port',
];

const MULTI_ORDER = ['ego_swarm', 'shared_field', 'formation'];

const state = {
  mode: 'single',
  planner: 'gcopter',
  multiMode: 'ego_swarm',
  map: 'official_forest',
  planners: {},
  multiModes: {},
  maps: {},
  clearLocalLogs: false,
  lang: localStorage.getItem('drone_dash_lang') || 'zh',
  running: false,
};

const el = (id) => document.getElementById(id);
const t = (key) => (I18N[state.lang] && I18N[state.lang][key]) || I18N.en[key] || key;

async function api(path, opts = {}) {
  const res = await fetch(path, {
    headers: { 'Content-Type': 'application/json' },
    ...opts,
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(data.error || res.statusText);
  return data;
}

function applyI18n() {
  document.documentElement.lang = state.lang === 'zh' ? 'zh-CN' : 'en';
  document.querySelectorAll('[data-i18n]').forEach((node) => {
    node.textContent = t(node.dataset.i18n);
  });
  document.querySelectorAll('[data-i18n-html]').forEach((node) => {
    node.innerHTML = t(node.dataset.i18nHtml);
  });
  document.querySelectorAll('.lang-btn').forEach((btn) => {
    btn.classList.toggle('active', btn.dataset.lang === state.lang);
  });
  document.title = state.lang === 'zh' ? '无人机仿真控制台' : 'Drone Sim Control';
  setRunningUI(state.running);
  if (Object.keys(state.planners).length) renderPlanners(state.planners);
  if (Object.keys(state.multiModes).length) renderMulti(state.multiModes);
  if (Object.keys(state.maps).length) renderMaps(state.maps);
  updateModeUI();
}

function setLang(lang) {
  if (!I18N[lang]) return;
  state.lang = lang;
  localStorage.setItem('drone_dash_lang', lang);
  applyI18n();
}

function updateModeUI() {
  document.querySelectorAll('.mode-chip').forEach((b) => {
    b.setAttribute('aria-checked', b.dataset.mode === state.mode ? 'true' : 'false');
  });
  el('singleBlock').hidden = state.mode !== 'single';
  el('multiBlock').hidden = state.mode !== 'multi';
  el('swarmPos').hidden = state.mode !== 'multi';
}

function renderPlanners(planners) {
  state.planners = planners || {};
  const grid = el('plannerGrid');
  grid.innerHTML = '';
  for (const key of ['homemade', 'ego', 'gcopter']) {
    if (!PLANNER_I18N[key]) continue;
    const keys = PLANNER_I18N[key];
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'planner-card';
    btn.dataset.planner = key;
    btn.innerHTML = `<span class="title">${t(keys.label)}</span><span class="desc">${t(keys.desc)}</span>`;
    btn.addEventListener('click', () => selectPlanner(key));
    grid.appendChild(btn);
  }
  selectPlanner(state.planner, false);
}

function renderMulti(modes) {
  state.multiModes = modes || {};
  const grid = el('multiGrid');
  grid.innerHTML = '';
  for (const key of MULTI_ORDER) {
    const meta = modes[key];
    if (!meta) continue;
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'planner-card';
    btn.dataset.multi = key;
    const label = state.lang === 'zh' ? meta.label_zh : meta.label_en;
    const desc = state.lang === 'zh' ? meta.desc_zh : meta.desc_en;
    btn.innerHTML = `<span class="title">${label}</span><span class="desc">${desc}</span>`;
    btn.addEventListener('click', () => selectMulti(key));
    grid.appendChild(btn);
  }
  selectMulti(state.multiMode, false);
}

function renderMaps(maps) {
  state.maps = maps || {};
  const grid = el('mapGrid');
  grid.innerHTML = '';
  for (const key of MAP_ORDER) {
    const meta = maps[key];
    if (!meta) continue;
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'map-card';
    btn.dataset.map = key;
    const familyKey = meta.family === 'official' ? 'familyOfficial' : 'familyHomemade';
    const label = state.lang === 'zh' ? meta.label_zh : meta.label_en;
    const desc = state.lang === 'zh' ? meta.desc_zh : meta.desc_en;
    btn.innerHTML = `
      <span class="family">${t(familyKey)}</span>
      <span class="title">${label}</span>
      <span class="desc">${desc}</span>`;
    btn.addEventListener('click', () => selectMap(key));
    grid.appendChild(btn);
  }
  selectMap(state.map, false);
}

function selectPlanner(key, sync = true) {
  state.planner = key;
  document.querySelectorAll('[data-planner]').forEach((card) => {
    card.setAttribute('aria-checked', card.dataset.planner === key ? 'true' : 'false');
  });
  if (sync) syncConfig();
}

function selectMulti(key, sync = true) {
  state.multiMode = key;
  document.querySelectorAll('[data-multi]').forEach((card) => {
    card.setAttribute('aria-checked', card.dataset.multi === key ? 'true' : 'false');
  });
  if (
    state.mode === 'multi' &&
    key === 'ego_swarm' &&
    !String(state.map).startsWith('official')
  ) {
    state.map = 'official_forest';
    selectMap(state.map, false);
  }
  if (sync) syncConfig();
}

function selectMap(key, sync = true) {
  state.map = key;
  document.querySelectorAll('.map-card').forEach((card) => {
    card.setAttribute('aria-checked', card.dataset.map === key ? 'true' : 'false');
  });
  if (sync) syncConfig();
}

function selectRunMode(mode, sync = true) {
  state.mode = mode;
  updateModeUI();
  if (mode === 'multi') {
    state.multiMode = state.multiMode || 'ego_swarm';
    state.map = state.map || 'official_forest';
  }
  if (sync) syncConfig();
}

function mapGoalDefaults() {
  const meta = state.maps[state.map] || {};
  return {
    goal_x: meta.goal_x ?? 15.0,
    goal_y: meta.goal_y ?? 0.0,
    goal_z: meta.goal_z ?? 1.0,
  };
}

function formConfig() {
  const goals = mapGoalDefaults();
  return {
    mode: state.mode,
    planner: state.planner,
    multi_mode: state.multiMode,
    num_drones: Number(el('numDrones').value) || 2,
    formation: el('formation').value,
    map: state.map,
    seed: Number(el('seed').value) || 1,
    max_vel: Number(el('maxVel').value) || 1.2,
    use_rviz: el('useRviz').checked,
    goal_x: goals.goal_x,
    goal_y: goals.goal_y,
    goal_z: goals.goal_z,
  };
}

async function syncConfig() {
  const data = await api('/api/config', { method: 'POST', body: JSON.stringify(formConfig()) });
  el('cmdPreview').textContent = data.cmd || '—';
}

function setRunningUI(running) {
  state.running = !!running;
  const pill = el('runPill');
  pill.dataset.state = running ? 'running' : 'idle';
  el('runLabel').textContent = running ? t('running') : t('idle');
  el('btnStart').disabled = running;
  el('btnStop').disabled = !running;
}

function fmt(n, digits = 2) {
  if (n === null || n === undefined || Number.isNaN(n)) return '—';
  return Number(n).toFixed(digits);
}

function applyStatus(st) {
  if (st.planners && !Object.keys(state.planners).length) renderPlanners(st.planners);
  if (st.multi_modes && !Object.keys(state.multiModes).length) renderMulti(st.multi_modes);
  if (st.maps && !Object.keys(state.maps).length) renderMaps(st.maps);

  setRunningUI(!!st.running);
  el('pid').textContent = st.pid ?? '—';
  el('uptime').textContent = st.running ? `${fmt(st.uptime_s, 0)} s` : '—';
  el('cmdPreview').textContent = st.cmd || '—';

  const o = st.odom;
  if (o) {
    el('pos').textContent = `${fmt(o.x)}, ${fmt(o.y)}, ${fmt(o.z)}`;
    el('vel').textContent = `${fmt(o.vx)}, ${fmt(o.vy)}, ${fmt(o.vz)}`;
  } else {
    el('pos').textContent = '—';
    el('vel').textContent = '—';
  }

  const swarm = st.swarm_odom || {};
  const keys = Object.keys(swarm).sort();
  if (keys.length) {
    el('swarmPos').hidden = false;
    el('swarmPos').textContent = keys.map((k) => {
      const p = swarm[k];
      return `${k}: ${fmt(p.x)}, ${fmt(p.y)}, ${fmt(p.z)}`;
    }).join('\n');
  }

  if (!state.clearLocalLogs && Array.isArray(st.logs)) {
    el('logView').textContent = st.logs.join('\n');
    el('logView').scrollTop = el('logView').scrollHeight;
  }

  if (st.config && !st.running && document.activeElement?.tagName !== 'INPUT') {
    const c = st.config;
    el('seed').value = c.seed ?? 1;
    el('maxVel').value = c.max_vel ?? 1.2;
    el('useRviz').checked = !!c.use_rviz;
    el('numDrones').value = c.num_drones ?? 2;
    el('formation').value = c.formation ?? 'v';
    if (c.mode) state.mode = c.mode;
    if (c.planner) state.planner = c.planner;
    if (c.multi_mode) state.multiMode = c.multi_mode;
    if (c.map) state.map = c.map;
    updateModeUI();
    selectPlanner(state.planner, false);
    selectMulti(state.multiMode, false);
    selectMap(state.map, false);
  }
}

async function poll() {
  try {
    applyStatus(await api('/api/status'));
  } catch (err) {
    el('runLabel').textContent = t('offline');
    el('runPill').dataset.state = 'idle';
  }
}

async function startSim() {
  state.clearLocalLogs = false;
  const data = await api('/api/start', { method: 'POST', body: JSON.stringify(formConfig()) });
  if (!data.ok) alert(data.error || t('startFailed'));
  await poll();
}

async function stopSim() {
  await api('/api/stop', { method: 'POST', body: '{}' });
  await poll();
}

async function restartSim() {
  state.clearLocalLogs = false;
  await api('/api/restart', { method: 'POST', body: JSON.stringify(formConfig()) });
  await poll();
}

document.querySelectorAll('.mode-chip').forEach((btn) => {
  btn.addEventListener('click', () => selectRunMode(btn.dataset.mode));
});

document.querySelectorAll('[data-preset]').forEach((btn) => {
  btn.addEventListener('click', () => {
    const p = btn.dataset.preset;
    if (p === 'cross') {
      selectRunMode('single', false);
      selectPlanner('gcopter', false);
      selectMap('official_forest', false);
      el('seed').value = 1;
    } else if (p === 'swarm') {
      selectRunMode('multi', false);
      selectMulti('ego_swarm', false);
      selectMap('official_forest', false);
      el('numDrones').value = 2;
      el('seed').value = 1;
    } else if (p === 'home') {
      selectRunMode('single', false);
      selectPlanner('homemade', false);
      selectMap('dense_field', false);
      el('seed').value = 42;
    }
    syncConfig();
  });
});

document.querySelectorAll('.lang-btn').forEach((btn) => {
  btn.addEventListener('click', () => setLang(btn.dataset.lang));
});

el('btnStart').addEventListener('click', () => startSim().catch((e) => alert(e.message)));
el('btnStop').addEventListener('click', () => stopSim().catch((e) => alert(e.message)));
el('btnRestart').addEventListener('click', () => restartSim().catch((e) => alert(e.message)));
el('btnClearLog').addEventListener('click', () => {
  state.clearLocalLogs = true;
  el('logView').textContent = '';
  setTimeout(() => { state.clearLocalLogs = false; }, 1500);
});

['seed', 'maxVel', 'useRviz', 'numDrones', 'formation'].forEach((id) => {
  el(id).addEventListener('change', () => syncConfig().catch(() => {}));
});

el('hostHint').textContent = location.host;
applyI18n();
poll();
setInterval(poll, 1000);
