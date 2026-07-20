/**
 * LiquidGlass — official site presets from liquid-glass.ybouane.com
 * Frosted Panel · Dark Glass · Button Mode
 *
 * Page cards / panels use WebGL LiquidGlass at night.
 * Top deck-bar stays CSS frost; pills inside (#deckLg) are LiquidGlass.
 * Sidebar stays CSS frost (separate root; keeps layout stable).
 */
import { LiquidGlass } from './vendor/liquidglass/index.js?v=sticky1';

/** Shared optical defaults from the Interactive Playground / README. */
const BASE = {
  floating: false,
  refraction: 0.69,
  chromAberration: 0.05,
  edgeHighlight: 0.05,
  specular: 0,
  fresnel: 1,
  distortion: 0,
  opacity: 1,
  saturation: 0,
  tintStrength: 0,
  shadowOpacity: 0.3,
  shadowSpread: 10,
  shadowOffsetY: 1,
  bevelMode: 0,
  zRadius: 40,
};

/** Official "Frosted Panel" recipe. */
const FROSTED = {
  ...BASE,
  button: false,
  blurAmount: 0.25,
  brightness: 0,
  cornerRadius: 30,
};

/** Official "Dark Glass" recipe. */
const DARK = {
  ...BASE,
  button: false,
  blurAmount: 0.25,
  brightness: -0.3,
  cornerRadius: 50,
};

/** Official "Button Mode" — hover brightens, press flattens bevel. */
const BUTTON = {
  ...BASE,
  button: true,
  blurAmount: 0.25,
  brightness: 0,
  cornerRadius: 24,
};

/**
 * Map / top-down shell — same Dark Glass family as planner cards,
 * with a slightly deeper bevel (calculator-style body).
 */
const CALCULATOR = {
  ...DARK,
  cornerRadius: 36,
  zRadius: 44,
  blurAmount: 0.28,
  brightness: -0.28,
  refraction: 0.72,
  edgeHighlight: 0.1,
  chromAberration: 0.04,
  shadowOpacity: 0.32,
  shadowSpread: 14,
  shadowOffsetY: 2,
};

const PAGE_ROOT_IDS = ['missionLgSingle', 'missionLgMulti', 'labLg'];
const LIQUID_PAGES = ['lab', 'single', 'multi'];
/** Keep sampling hot through trackpad inertia (wheel events stop early). */
const MOTION_MS = 700;

let instance = null;
let chromeInstance = null;
let bootToken = 0;
let chromeToken = 0;
let refreshTimer = null;
let chromeTimer = null;
let preloadTimer = 0;
let preloadBusy = false;
let motionListening = false;
let motionRaf = 0;
let lastGlassKey = '';
let lastChromeKey = '';
let scrollEndTimer = 0;
/** @type {Map<string, { instance: object, key: string }>} */
const pageCache = new Map();
let activeLiquidPage = '';

function prefersReducedMotion() {
  try {
    return window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  } catch (e) {
    return false;
  }
}

function isNight() {
  return document.documentElement.getAttribute('data-ambiance') !== 'day';
}

function appBgHome() {
  return document.querySelector('.shell') || document.body;
}

function getAppBg() {
  return document.getElementById('appBg');
}

function restoreAppBg() {
  const img = getAppBg();
  const home = appBgHome();
  if (!img || !home || img.parentElement === home) return;
  const veil = home.querySelector(':scope > .app-bg-veil');
  if (veil) home.insertBefore(img, veil);
  else home.insertBefore(img, home.firstChild);
}

/**
 * Keep #appBg under .shell so frost can see the wallpaper.
 * LiquidGlass only samples children of its root — give each root a clone.
 */
function adoptAppBg(root) {
  const srcImg = getAppBg();
  if (!srcImg || !root) return null;
  restoreAppBg();
  let sample = root.querySelector(':scope > img.app-bg-sample');
  if (!sample) {
    sample = document.createElement('img');
    sample.className = 'app-bg app-bg-sample';
    sample.alt = '';
    sample.setAttribute('aria-hidden', 'true');
    sample.decoding = 'async';
    root.insertBefore(sample, root.firstChild);
  }
  // Always mirror #appBg — relative vs absolute src must not block updates.
  const nextSrc = srcImg.currentSrc || srcImg.getAttribute('src') || srcImg.src;
  if (nextSrc) sample.src = nextSrc;
  return sample;
}

/** Push current wallpaper into every liquid sample (without full rebuild). */
function syncAppBgSamples() {
  const srcImg = getAppBg();
  if (!srcImg) return;
  const nextSrc = srcImg.currentSrc || srcImg.getAttribute('src') || srcImg.src;
  if (!nextSrc) return;
  document.querySelectorAll('img.app-bg-sample').forEach((sample) => {
    sample.src = nextSrc;
  });
}

function clearLiquidClasses(root) {
  if (!root) return;
  root.querySelectorAll(':scope > img.app-bg-sample').forEach((n) => n.remove());
  root.querySelectorAll('.is-liquid-glass').forEach((el) => {
    el.classList.remove('is-liquid-glass');
  });
}

function clearAllPageLiquidClasses() {
  PAGE_ROOT_IDS.forEach((id) => clearLiquidClasses(document.getElementById(id)));
}

/** Restore CSS frost (only for teardown / day mode — not used on page switch). */
function frostGlasses(glassElements) {
  (glassElements || []).forEach((el) => {
    if (el) el.classList.remove('is-liquid-glass');
  });
}

function glassesOf(inst) {
  if (!inst || !inst.glassSet) return [];
  return Array.from(inst.glassSet);
}

/**
 * After a cold boot: paint 1–2 frames under frost, then flip to glass.
 * Cached switches skip this — they keep the frozen last WebGL frame.
 */
async function sealFirstFrame(inst, glassElements) {
  const els = glassElements && glassElements.length ? glassElements : glassesOf(inst);
  frostGlasses(els);
  if (!inst) return;
  try { inst.markChanged(); } catch (e) { /* ignore */ }
  await new Promise((r) => requestAnimationFrame(r));
  try { inst.markChanged(); } catch (e) { /* ignore */ }
  await new Promise((r) => requestAnimationFrame(r));
  els.forEach((el) => {
    if (el && el.isConnected) el.classList.add('is-liquid-glass');
  });
  try { inst.markChanged(); } catch (e) { /* ignore */ }
}

function pauseLiquid(inst) {
  if (!inst) return;
  // Keep is-liquid-glass + last canvas frame frozen (instant final look on resume).
  if (!inst._running) return;
  inst._running = false;
  if (inst._rafId) {
    cancelAnimationFrame(inst._rafId);
    inst._rafId = 0;
  }
}

function resumeLiquid(inst) {
  if (!inst || inst._running) return;
  inst._running = true;
  inst._globalDirty = true;
  try { inst.markChanged(); } catch (e) { /* ignore */ }
  inst._rafId = requestAnimationFrame(() => {
    try { inst._renderLoop(); } catch (e) { /* ignore */ }
  });
}

function destroyCachedPage(page) {
  const entry = pageCache.get(page);
  if (!entry) return;
  try { pauseLiquid(entry.instance); } catch (e) { /* ignore */ }
  try { entry.instance.destroy(); } catch (e) { /* ignore */ }
  pageCache.delete(page);
  const root = pageRoot(page);
  if (root) clearLiquidClasses(root);
  if (activeLiquidPage === page) {
    activeLiquidPage = '';
    instance = null;
    lastGlassKey = '';
  }
}

function destroyPageLiquid() {
  for (const page of [...pageCache.keys()]) destroyCachedPage(page);
  destroyMapLiquid();
  instance = null;
  activeLiquidPage = '';
  lastGlassKey = '';
  clearAllPageLiquidClasses();
  restoreAppBg();
}

function waitForImage(img) {
  if (!img) return Promise.resolve();
  if (img.complete && img.naturalWidth > 0) return Promise.resolve();
  return new Promise((resolve) => {
    const done = () => resolve();
    img.addEventListener('load', done, { once: true });
    img.addEventListener('error', done, { once: true });
    setTimeout(done, 1200);
  });
}

function pulseOne(inst, ms = MOTION_MS, dirtyAll = false) {
  if (!inst) return;
  try {
    if (typeof inst.keepMotionHot === 'function' && !dirtyAll) {
      inst.keepMotionHot(ms);
    } else if (typeof inst.pulseMotion === 'function') {
      inst.pulseMotion(ms, dirtyAll);
    }
  } catch (e) { /* ignore */ }
}

function pulseMotion(ms = MOTION_MS, dirtyAll = false) {
  pulseOne(instance, ms, dirtyAll);
  pulseOne(chromeInstance, ms, dirtyAll);
}

/** Sticky shells don't move in viewport — background / cards scroll under them. */
function dirtyStickyGlasses() {
  if (!instance || !instance.root) return;
  try {
    const dirty = instance._glassDirty;
    instance.root.querySelectorAll(
      '.is-liquid-glass[data-liquid-sticky="1"], #plannerShell.is-liquid-glass, #multiBlock.is-liquid-glass',
    ).forEach((el) => {
      if (dirty && typeof dirty.add === 'function') dirty.add(el);
      else instance.markChanged(el);
    });
  } catch (e) { /* ignore */ }
}

/**
 * Keep the shader loop hot and dirty visible glasses every scroll tick
 * so refraction stays real-time (no 1–2s catch-up queue).
 */
function kickMotion() {
  if (!instance && !chromeInstance) return;
  if (motionRaf) return;
  motionRaf = requestAnimationFrame(() => {
    motionRaf = 0;
    pulseMotion(MOTION_MS, false);
    dirtyStickyGlasses();
    // Force visible panel refresh this frame.
    if (instance && typeof instance.markChanged === 'function') {
      try {
        instance.root?.querySelectorAll('.is-liquid-glass').forEach((el) => {
          const r = el.getBoundingClientRect();
          const vh = window.innerHeight || 0;
          if (r.bottom > -40 && r.top < vh + 40) instance.markChanged(el);
        });
      } catch (e) { /* ignore */ }
    }
    if (scrollEndTimer) clearTimeout(scrollEndTimer);
    scrollEndTimer = setTimeout(() => {
      scrollEndTimer = 0;
      if (!instance) return;
      pulseMotion(120, false);
      try { instance.markChanged(); } catch (e) { /* ignore */ }
    }, 160);
  });
}

function bindMotionListeners() {
  if (motionListening) return;
  motionListening = true;
  // wheel bubbles; scroll does NOT — must listen on the real scroll containers.
  window.addEventListener('wheel', kickMotion, { capture: true, passive: true });
  window.addEventListener('touchmove', kickMotion, { capture: true, passive: true });
  const scrollRoots = [
    document.getElementById('workspace'),
    document.querySelector('.rail__scroll'),
  ].filter(Boolean);
  scrollRoots.forEach((node) => {
    node.addEventListener('scroll', kickMotion, { passive: true });
  });
}

function glassKey(root, glassElements) {
  // Stable ids only — className changes when is-liquid-glass is toggled.
  return `${root && root.id}:${glassElements.map((el, i) => (
    el.id
    || el.dataset.planner
    || el.dataset.map
    || el.dataset.liquidSlot
    || `${el.tagName}-${el.dataset.liquidRole || 'g'}-${i}`
  )).join('|')}`;
}

function pageRoot(page) {
  if (page === 'single') return document.getElementById('missionLgSingle');
  if (page === 'multi') return document.getElementById('missionLgMulti');
  if (page === 'lab') return document.getElementById('labLg');
  return null;
}

function isButtonGlass(el) {
  const role = el.dataset.liquidRole;
  if (role === 'button') return true;
  if (role === 'panel' || role === 'chip') return false;
  return el.matches('button, [role="button"]')
    || el.classList.contains('home-tile')
    || el.classList.contains('chrome-seg')
    || el.classList.contains('bg-picker');
}

function configFor(el) {
  const night = isNight();
  const panelBase = night ? { ...DARK } : { ...FROSTED };
  const buttonBase = night
    ? { ...DARK, button: true }
    : { ...BUTTON };

  // Status chips (SINGLE / 空闲): Dark Glass capsule.
  if (el.classList.contains('page-chip') || el.classList.contains('live-pill')
      || el.dataset.liquidRole === 'chip') {
    return {
      ...(night ? DARK : FROSTED),
      button: false,
      cornerRadius: 999,
      zRadius: 22,
      blurAmount: 0.22,
      shadowOpacity: 0.2,
      shadowSpread: 8,
      shadowOffsetY: 1,
    };
  }
  // 背景 / 亮暗 / EN·中文: Button Mode capsules.
  if (el.classList.contains('chrome-pill') || el.classList.contains('chrome-seg')
      || el.classList.contains('bg-picker')) {
    return {
      ...buttonBase,
      cornerRadius: 999,
      zRadius: 22,
      blurAmount: 0.22,
      shadowOpacity: 0.22,
      shadowSpread: 8,
    };
  }

  if (isButtonGlass(el)) {
    if (el.classList.contains('home-tile')) {
      return { ...buttonBase, cornerRadius: 40, zRadius: 40 };
    }
    // Lighter cards — many on one page; keeps scroll rendering continuous.
    // cornerRadius must match CSS for button-mode surfaces.
    return {
      ...buttonBase,
      cornerRadius: 28,
      zRadius: 24,
      blurAmount: 0.18,
      shadowOpacity: 0.18,
      shadowSpread: 6,
    };
  }
  if (el.classList.contains('home-lg__dock')) {
    // Must match CSS .home-lg__dock border-radius (40) — was 50 vs 26 mismatch.
    return night
      ? { ...DARK, cornerRadius: 40, zRadius: 36 }
      : { ...FROSTED, cornerRadius: 40, zRadius: 36 };
  }
  if (el.id === 'xyTrackPanel' || el.classList.contains('map-panel')
      || el.classList.contains('map-panel-fields') || el.classList.contains('map-panel--calc')) {
    // Calculator body: dark glass shell; nested cards stay CSS “keys”.
    return night
      ? { ...CALCULATOR }
      : { ...FROSTED, cornerRadius: 36, zRadius: 40, blurAmount: 0.28 };
  }
  if (el.classList.contains('lab-panel')) {
    return night
      ? { ...DARK, cornerRadius: 26, zRadius: 36 }
      : { ...FROSTED, cornerRadius: 26, zRadius: 36 };
  }
  // mission-shell — match CSS border-radius 28.
  if (el.classList.contains('mission-shell')) {
    return { ...panelBase, cornerRadius: 28, zRadius: 32 };
  }
  return { ...panelBase, cornerRadius: 18, zRadius: 28 };
}

function collectGlasses(root, page) {
  if (!root) return [];
  if (page === 'lab') {
    return Array.from(root.querySelectorAll(':scope > [data-liquid="1"]'));
  }
  // Stable planner / multi shells only. Map lives in separate hosts so
  // single↔multi reparent never busts this page cache (keeps last frame).
  return Array.from(root.querySelectorAll(
    ':scope > .mission-shell[data-liquid="1"]:not(.map-panel):not(.map-panel-fields)',
  ));
}

const MAP_HOST_IDS = ['mapPanelHost', 'mapFieldsHost', 'xyTrackHost'];
/** @type {Map<string, { instance: object, key: string }>} */
const mapCache = new Map();
let mapBootToken = 0;

function mapGlassOf(host) {
  if (!host) return null;
  return host.querySelector(':scope > [data-liquid="1"]');
}

function destroyMapLiquid() {
  for (const [id, entry] of [...mapCache.entries()]) {
    try { pauseLiquid(entry.instance); } catch (e) { /* ignore */ }
    try { entry.instance.destroy(); } catch (e) { /* ignore */ }
    mapCache.delete(id);
    const host = document.getElementById(id);
    if (host) {
      const g = mapGlassOf(host);
      if (g) g.classList.remove('is-liquid-glass');
      host.querySelectorAll(':scope > img.app-bg-sample').forEach((n) => n.remove());
    }
  }
}

async function bootMapHost(host, isValid = () => true) {
  const glass = mapGlassOf(host);
  if (!host || !glass || host.hidden) return null;
  const key = glassKey(host, [glass]);
  const cached = mapCache.get(host.id);
  if (cached && cached.key === key && cached.instance.glassSet.has(glass)) {
    adoptAppBg(host);
    syncAppBgSamples();
    glass.classList.add('is-liquid-glass');
    try { cached.instance.markChanged(); } catch (e) { /* ignore */ }
    return cached.instance;
  }
  if (cached) {
    try { cached.instance.destroy(); } catch (e) { /* ignore */ }
    mapCache.delete(host.id);
    glass.classList.remove('is-liquid-glass');
  }
  const next = await bootRoot(host, [glass], { ...CALCULATOR }, isValid);
  if (!next) return null;
  mapCache.set(host.id, { instance: next, key });
  return next;
}

/**
 * Map deck LiquidGlass — independent of page cache so frozen frames survive
 * moving hosts between single and multi.
 */
async function initMapLiquid(active) {
  if (prefersReducedMotion() || !isNight()) {
    destroyMapLiquid();
    return;
  }
  if (active !== 'single' && active !== 'multi') {
    // Leave canvases + last frames; just pause RAF.
    for (const entry of mapCache.values()) pauseLiquid(entry.instance);
    return;
  }
  const token = ++mapBootToken;
  const hosts = MAP_HOST_IDS.map((id) => document.getElementById(id)).filter(Boolean);
  for (const host of hosts) {
    if (token !== mapBootToken) return;
    if (host.hidden) continue;
    try {
      const inst = await bootMapHost(host, () => token === mapBootToken);
      if (inst) {
        resumeLiquid(inst);
        forceVisiblePass(inst);
        pulseOne(inst, MOTION_MS, false);
      }
    } catch (err) {
      console.warn('[liquid-boot] map host failed:', host.id, err);
    }
  }
}

function refreshMapLiquid() {
  const page = document.documentElement.getAttribute('data-page') || '';
  initMapLiquid(page);
}

function forceVisiblePass(inst) {
  if (!inst) return;
  try { inst.markChanged(); } catch (e) { /* ignore */ }
  requestAnimationFrame(() => {
    try { inst && inst.markChanged(); } catch (e) { /* ignore */ }
  });
}

async function bootRoot(root, glassElements, defaults, isValid = () => true) {
  if (!root || !glassElements.length) return null;
  const sampleBg = adoptAppBg(root);
  await waitForImage(sampleBg);
  if (!isValid()) {
    restoreAppBg();
    return null;
  }

  // Keep CSS frost visible during init — only flip to WebGL after first frame.
  glassElements.forEach((el) => {
    el.dataset.config = JSON.stringify(configFor(el));
  });

  await new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r)));
  if (!isValid()) {
    restoreAppBg();
    return null;
  }

  let next;
  try {
    next = await LiquidGlass.init({
      root,
      glassElements,
      defaults,
    });
  } catch (err) {
    restoreAppBg();
    throw err;
  }

  if (!isValid()) {
    try { next.destroy(); } catch (e) { /* ignore */ }
    restoreAppBg();
    return null;
  }

  // Cold boot only: seal first WebGL frame, then show glass (no black hole).
  await sealFirstFrame(next, glassElements);
  if (!isValid()) {
    try { next.destroy(); } catch (e) { /* ignore */ }
    frostGlasses(glassElements);
    restoreAppBg();
    return null;
  }
  return next;
}

async function initLiquidForPage(page) {
  if (!LIQUID_PAGES.includes(page)) {
    if (activeLiquidPage) {
      const prev = pageCache.get(activeLiquidPage);
      if (prev) pauseLiquid(prev.instance);
      activeLiquidPage = '';
      instance = null;
      lastGlassKey = '';
    }
    initMapLiquid(page);
    return;
  }
  if (prefersReducedMotion() || !isNight()) {
    destroyPageLiquid();
    return;
  }

  const root = pageRoot(page);
  if (!root) return;
  const pageEl = root.closest('.page');
  if (pageEl && !pageEl.classList.contains('is-active') && !pageEl.classList.contains('page--liquid-preload')) {
    return;
  }

  const glassElements = collectGlasses(root, page);
  if (!glassElements.length) {
    destroyCachedPage(page);
    restoreAppBg();
    return;
  }

  const key = glassKey(root, glassElements);

  // Pause previous page — keep its instance warm in cache.
  if (activeLiquidPage && activeLiquidPage !== page) {
    const prev = pageCache.get(activeLiquidPage);
    if (prev) pauseLiquid(prev.instance);
  }

  const cached = pageCache.get(page);
  const cacheOk = cached
    && cached.key === key
    && glassElements.every((el) => (
      el.parentElement === root
      && cached.instance.glassSet
      && cached.instance.glassSet.has(el)
    ));
  if (cacheOk) {
    instance = cached.instance;
    lastGlassKey = key;
    activeLiquidPage = page;
    adoptAppBg(root);
    syncAppBgSamples();
    bindMotionListeners();
    // Instant final look: last WebGL frame is still on the canvases.
    glassElements.forEach((el) => el.classList.add('is-liquid-glass'));
    resumeLiquid(instance);
    forceVisiblePass(instance);
    pulseMotion(MOTION_MS);
    schedulePreload(page);
    initMapLiquid(page);
    return;
  }

  // Stale cache (rare for stable shells) — rebuild this page only.
  if (cached) destroyCachedPage(page);

  const token = ++bootToken;
  preloadToken += 1; // cancel idle preloads so they don't race the active boot

  try {
    const next = await bootRoot(root, glassElements, { ...DARK }, () => token === bootToken);
    if (!next) {
      restoreAppBg();
      return;
    }
    pageCache.set(page, { instance: next, key });
    instance = next;
    lastGlassKey = key;
    activeLiquidPage = page;
    bindMotionListeners();
    forceVisiblePass(instance);
    pulseMotion(MOTION_MS);
    schedulePreload(page);
    initMapLiquid(page);
  } catch (err) {
    console.warn('[liquid-boot] page LiquidGlass init failed:', err);
    destroyCachedPage(page);
    restoreAppBg();
  }
}

/** Off-screen measurable layout so preload can capture real glass sizes. */
function withPagePreload(pageEl, fn) {
  if (!pageEl) return Promise.resolve();
  pageEl.classList.add('page--liquid-preload');
  return Promise.resolve()
    .then(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))))
    .then(fn)
    .finally(() => {
      pageEl.classList.remove('page--liquid-preload');
    });
}

let preloadToken = 0;

async function preloadPage(page) {
  if (!LIQUID_PAGES.includes(page)) return;
  if (pageCache.has(page)) return;
  if (prefersReducedMotion() || !isNight()) return;
  if (page === activeLiquidPage) return;

  const root = pageRoot(page);
  const pageEl = root && root.closest('.page');
  if (!root || !pageEl) return;

  const token = ++preloadToken;
  await withPagePreload(pageEl, async () => {
    if (pageCache.has(page) || page === activeLiquidPage) return;
    const glassElements = collectGlasses(root, page);
    if (!glassElements.length) return;
    const key = glassKey(root, glassElements);
    try {
      const next = await bootRoot(
        root,
        glassElements,
        { ...DARK },
        () => token === preloadToken && !pageCache.has(page) && page !== activeLiquidPage,
      );
      if (!next) return;
      if (pageCache.has(page)) {
        try { next.destroy(); } catch (e) { /* ignore */ }
        clearLiquidClasses(root);
        return;
      }
      pauseLiquid(next);
      // Keep sealed glass + last frame for instant show on first visit.
      pageCache.set(page, { instance: next, key });
    } catch (err) {
      console.warn('[liquid-boot] preload failed:', page, err);
      clearLiquidClasses(root);
    }
  });

  const active = document.documentElement.getAttribute('data-page') || '';
  const activeRoot = pageRoot(active);
  if (activeRoot) adoptAppBg(activeRoot);
  else restoreAppBg();
}

function schedulePreload(exceptPage) {
  if (preloadTimer) {
    try { cancelIdleCallback(preloadTimer); } catch (e) { clearTimeout(preloadTimer); }
    preloadTimer = 0;
  }
  const run = () => {
    preloadTimer = 0;
    if (preloadBusy) return;
    preloadBusy = true;
    (async () => {
      try {
        for (const page of LIQUID_PAGES) {
          if (page === exceptPage) continue;
          if (pageCache.has(page)) continue;
          await preloadPage(page);
        }
      } finally {
        preloadBusy = false;
      }
    })();
  };
  // Eager warm — don't wait for long idle; first revisit must feel instant.
  preloadTimer = setTimeout(run, 120);
}

function destroyChromeLiquid() {
  if (chromeInstance) {
    try { chromeInstance.destroy(); } catch (e) { /* ignore */ }
    chromeInstance = null;
  }
  lastChromeKey = '';
  const root = document.getElementById('deckLg');
  if (root) clearLiquidClasses(root);
  // Sidebar stays CSS frost — never WebGL.
  const rail = document.getElementById('chromeLg');
  if (rail) {
    rail.querySelectorAll('.is-liquid-glass').forEach((el) => el.classList.remove('is-liquid-glass'));
  }
}

/** Small pills / segs inside the top deck bar (not the bar itself). */
function collectDeckGlasses(root) {
  if (!root) return [];
  return Array.from(root.querySelectorAll(
    ':scope > .page-chip[data-liquid="1"],'
    + ' :scope > .live-pill[data-liquid="1"],'
    + ' :scope > .chrome-pill[data-liquid="1"],'
    + ' :scope > .chrome-seg[data-liquid="1"],'
    + ' :scope > .bg-picker[data-liquid="1"]',
  ));
}

/**
 * Top deck-bar keeps CSS frost; only nested chips / buttons get LiquidGlass.
 */
async function initChromeLiquid() {
  if (prefersReducedMotion() || !isNight()) {
    destroyChromeLiquid();
    return;
  }

  const root = document.getElementById('deckLg');
  if (!root) return;
  const glassElements = collectDeckGlasses(root);
  if (!glassElements.length) {
    destroyChromeLiquid();
    return;
  }

  const key = glassKey(root, glassElements);
  if (chromeInstance && chromeInstance.root === root && key === lastChromeKey) {
    adoptAppBg(root);
    syncAppBgSamples();
    forceVisiblePass(chromeInstance);
    pulseOne(chromeInstance, 240, false);
    return;
  }

  const token = ++chromeToken;
  destroyChromeLiquid();

  try {
    const sampleBg = adoptAppBg(root);
    await waitForImage(sampleBg);
    if (token !== chromeToken) {
      clearLiquidClasses(root);
      restoreAppBg();
      return;
    }

    glassElements.forEach((el) => {
      el.dataset.config = JSON.stringify(configFor(el));
      el.classList.add('is-liquid-glass');
    });

    await new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r)));
    if (token !== chromeToken) {
      clearLiquidClasses(root);
      restoreAppBg();
      return;
    }

    const next = await LiquidGlass.init({
      root,
      glassElements,
      defaults: { ...DARK },
    });
    if (token !== chromeToken) {
      try { next.destroy(); } catch (e) { /* ignore */ }
      clearLiquidClasses(root);
      restoreAppBg();
      return;
    }
    chromeInstance = next;
    lastChromeKey = key;
    bindMotionListeners();
    forceVisiblePass(chromeInstance);
    pulseOne(chromeInstance, 240, false);
  } catch (err) {
    console.warn('[liquid-boot] deck LiquidGlass init failed:', err);
    clearLiquidClasses(root);
    restoreAppBg();
    lastChromeKey = '';
  }
}

function refreshMissionLiquid(immediate = false) {
  const go = () => {
    refreshTimer = null;
    const page = document.documentElement.getAttribute('data-page') || '';
    initLiquidForPage(page);
  };
  if (immediate) {
    if (refreshTimer) clearTimeout(refreshTimer);
    refreshTimer = null;
    go();
    return;
  }
  if (refreshTimer) clearTimeout(refreshTimer);
  refreshTimer = setTimeout(go, 40);
}

function refreshChromeLiquid() {
  if (chromeTimer) clearTimeout(chromeTimer);
  chromeTimer = setTimeout(() => {
    chromeTimer = null;
    initChromeLiquid();
  }, 60);
}

function markMissionLiquidChanged(target) {
  if (!instance) return;
  try {
    if (target) {
      instance.markChanged(target);
      return;
    }
    const root = instance.root;
    if (!root) return;
    const page = document.documentElement.getAttribute('data-page') || '';
    collectGlasses(root, page).forEach((el) => instance.markChanged(el));
  } catch (e) { /* ignore */ }
}

function markMapLiquidChanged() {
  const track = document.getElementById('xyTrackPanel');
  const host = document.getElementById('xyTrackHost');
  const entry = host && mapCache.get(host.id);
  if (entry && entry.instance) {
    try { entry.instance.markChanged(track || undefined); } catch (e) { /* ignore */ }
    return;
  }
  markMissionLiquidChanged(track);
}

window.refreshMissionLiquid = refreshMissionLiquid;
window.refreshChromeLiquid = refreshChromeLiquid;
window.refreshMapLiquid = refreshMapLiquid;
window.markMissionLiquidChanged = markMissionLiquidChanged;
window.markMapLiquidChanged = markMapLiquidChanged;
window.invalidateLiquidPage = destroyCachedPage;

document.addEventListener('DOMContentLoaded', () => {
  refreshChromeLiquid();
  // First paint with CSS frost, then LiquidGlass — avoids click jank.
  requestAnimationFrame(() => {
    requestAnimationFrame(() => refreshMissionLiquid(true));
  });
});
document.addEventListener('drone-page-changed', (ev) => {
  const page = (ev.detail && ev.detail.page) || '';
  // Cached page already has a sealed last WebGL frame — show it immediately.
  if (LIQUID_PAGES.includes(page) && pageCache.has(page)) {
    refreshMissionLiquid(true);
    return;
  }
  // Cold boot: allow one paint of CSS frost, then seal first glass frame.
  requestAnimationFrame(() => {
    requestAnimationFrame(() => refreshMissionLiquid(true));
  });
});
document.addEventListener('drone-ambiance-changed', () => {
  destroyPageLiquid();
  refreshChromeLiquid();
  requestAnimationFrame(() => {
    requestAnimationFrame(() => refreshMissionLiquid(true));
  });
});
document.addEventListener('drone-bg-changed', () => {
  syncAppBgSamples();
  lastChromeKey = '';
  destroyPageLiquid();
  refreshChromeLiquid();
  requestAnimationFrame(() => {
    requestAnimationFrame(() => refreshMissionLiquid(true));
  });
});
