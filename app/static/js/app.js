const appState = {
  connected: false,
  calibrated: false,
  running: false,
  paused: false,
  status: "Not connected",
  selectedColors: [],
  latestGcode: [],
  latestPreview: [],
  latestMaskPreview: null,
  latestSummary: null,
  latestAnalysis: null,
  latestMask: null,
  latestRegions: null,
  progressDone: 0,
  progressTotal: 0,
  runStartedAt: null,
  pauseStartedAt: null,
  pausedDurationSeconds: 0,
  generateRequestInFlight: false,
  calibrationRequestInFlight: false,
};

const previewViews = {
  mask: null,
  flat: null,
};

const previewData = {
  maskImage: null,
  flatPaths: [],
};

const PREVIEW_EMPTY_TEXT = {
  mask: "Generate a mask by selecting colors and creating G-code.",
  flat: "Generate G-code to inspect the flattened toolpath map.",
};

const FLAT_WORLD_LIMITS = { minX: -180, minY: -45, maxX: 180, maxY: 45 };
const FLAT_STROKE_SCREEN_MIN = 1.1;
const FLAT_STROKE_SCREEN_MAX = 3.6;

const SERVER_DEFAULTS = window.SERVER_DEFAULTS || {
  penUpS: 575,
  penDownS: 700,
  penUpDwellMs: 30,
  penDownDwellMs: 60,
  servoRampEnabled: true,
  servoRampStep: 20,
  servoRampDelayMs: 10,
};

const DERIVED_PEN_FIELDS = {
  infillSpacingMm: (penThickness) => penThickness,
  minFillWidthMm: (penThickness) => penThickness,
  minFillAreaMm2: (penThickness) => penThickness * penThickness,
};

function byId(id) {
  return document.getElementById(id);
}

function getNum(id) {
  return parseFloat(byId(id)?.value || "0");
}

function getInt(id) {
  return parseInt(byId(id)?.value || "0", 10);
}

function getBool(id) {
  return Boolean(byId(id)?.checked);
}

function getValue(id) {
  return byId(id)?.value?.trim() || "";
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");
}

function formatNumber(value) {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric.toLocaleString() : "-";
}

function formatDuration(totalSeconds) {
  const numeric = Number(totalSeconds);
  if (!Number.isFinite(numeric) || numeric < 0) return "--:--";
  const wholeSeconds = Math.round(numeric);
  const hours = Math.floor(wholeSeconds / 3600);
  const minutes = Math.floor((wholeSeconds % 3600) / 60);
  const seconds = wholeSeconds % 60;
  if (hours > 0) return `${hours}:${String(minutes).padStart(2, "0")}:${String(seconds).padStart(2, "0")}`;
  return `${String(minutes).padStart(2, "0")}:${String(seconds).padStart(2, "0")}`;
}

function appendLog(text) {
  const log = byId("log");
  if (!log) return;
  const now = new Date().toLocaleTimeString();
  log.textContent += `[${now}] ${text}\n`;
  log.scrollTop = log.scrollHeight;
}

function showToast(message, tone = "info") {
  const stack = byId("toastStack");
  if (!stack) return;
  const toast = document.createElement("div");
  toast.className = `toast toast--${tone}`;
  toast.textContent = message;
  stack.appendChild(toast);
  window.setTimeout(() => {
    toast.remove();
  }, 3200);
}

function roundTo(value, decimals) {
  const factor = 10 ** decimals;
  return Math.round(value * factor) / factor;
}

function syncDerivedPenFields(force = false) {
  const penThickness = Math.max(0, getNum("lineThicknessMm"));
  for (const [id, compute] of Object.entries(DERIVED_PEN_FIELDS)) {
    const input = byId(id);
    if (!input) continue;
    if (!force && input.dataset.userOverride === "1") continue;
    const decimals = id === "minFillAreaMm2" ? 3 : 2;
    input.value = String(roundTo(compute(penThickness), decimals));
    input.dataset.userOverride = "0";
  }

  const summary = byId("derivedPenSummary");
  if (!summary) return;
  const outlineInset = roundTo(penThickness / 2, 3);
  const infillSpacing = roundTo(DERIVED_PEN_FIELDS.infillSpacingMm(penThickness), 3);
  const minFillWidth = roundTo(DERIVED_PEN_FIELDS.minFillWidthMm(penThickness), 3);
  const minFillArea = roundTo(DERIVED_PEN_FIELDS.minFillAreaMm2(penThickness), 3);
  summary.textContent = `Auto defaults: outline inset ${outlineInset} mm, infill spacing ${infillSpacing} mm, min fill width ${minFillWidth} mm, min fill area ${minFillArea} mm^2.`;
}

function setupDerivedPenFieldSync() {
  byId("lineThicknessMm")?.addEventListener("input", () => syncDerivedPenFields(false));
  for (const id of Object.keys(DERIVED_PEN_FIELDS)) {
    const input = byId(id);
    if (!input) continue;
    input.addEventListener("input", () => {
      input.dataset.userOverride = input.value.trim() === "" ? "0" : "1";
    });
    input.addEventListener("blur", () => {
      if (input.value.trim() === "") syncDerivedPenFields(false);
    });
  }
}

function resetServoUiDefaults() {
  byId("penUpS").value = SERVER_DEFAULTS.penUpS;
  byId("penDownS").value = SERVER_DEFAULTS.penDownS;
  byId("penUpDwellMs").value = SERVER_DEFAULTS.penUpDwellMs;
  byId("penDownDwellMs").value = SERVER_DEFAULTS.penDownDwellMs;
  byId("servoRampEnabled").checked = SERVER_DEFAULTS.servoRampEnabled;
  byId("servoRampStep").value = SERVER_DEFAULTS.servoRampStep;
  byId("servoRampDelayMs").value = SERVER_DEFAULTS.servoRampDelayMs;
  byId("rawCommand").value = `M3 S${SERVER_DEFAULTS.penUpS}`;
  appendLog(`Reset servo UI fields to server defaults S${SERVER_DEFAULTS.penUpS}/S${SERVER_DEFAULTS.penDownS}.`);
}

function appendServoSettings(target) {
  target.servo_ramp_enabled = getBool("servoRampEnabled");
  target.servo_ramp_step = getInt("servoRampStep");
  target.servo_ramp_delay_ms = getNum("servoRampDelayMs");
  target.pen_up_dwell_ms = getNum("penUpDwellMs");
  target.pen_down_dwell_ms = getNum("penDownDwellMs");
  return target;
}

async function readJsonResponse(res) {
  const rawText = await res.text();
  try {
    return { json: JSON.parse(rawText), rawText };
  } catch (_) {
    return { json: null, rawText };
  }
}

async function api(url, data = {}, options = {}) {
  const { successMessage = "", errorPrefix = "ERROR", refresh = true } = options;
  try {
    const res = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(data),
    });
    const { json, rawText } = await readJsonResponse(res);
    if (!json) {
      appendLog(`${errorPrefix}: Non-JSON response from ${url}`);
      if (rawText) appendLog(rawText.slice(0, 600));
      showToast(`${errorPrefix}: request failed`, "error");
      return { ok: false, error: "Non-JSON response" };
    }
    if (json.ok) {
      if (json.command) appendLog(`> ${json.command}`);
      if (json.response) appendLog(`< ${json.response}`);
      if (successMessage) showToast(successMessage, "success");
    } else {
      appendLog(`${errorPrefix}: ${json.error}`);
      showToast(json.error || `${errorPrefix}`, "error");
    }
    if (refresh) await refreshState();
    return json;
  } catch (err) {
    appendLog(`${errorPrefix}: ${err}`);
    showToast(String(err), "error");
    return { ok: false, error: String(err) };
  }
}

async function refreshState() {
  try {
    const res = await fetch("/state");
    const state = await res.json();
    appState.connected = Boolean(state.connected);
    appState.calibrated = Boolean(state.calibrated);
    appState.running = Boolean(state.running);
    appState.paused = Boolean(state.paused);
    appState.status = state.status || "Idle";
    appState.progressDone = Number(state.progress_done || 0);
    appState.progressTotal = Number(state.progress_total || 0);
    appState.runStartedAt = state.run_started_at ? Number(state.run_started_at) : null;
    appState.pauseStartedAt = state.pause_started_at ? Number(state.pause_started_at) : null;
    appState.pausedDurationSeconds = Number(state.paused_duration_seconds || 0);

    const serialStatus = byId("serialStatus");
    serialStatus.textContent = appState.connected ? "Connected" : "Disconnected";
    serialStatus.className = `status-chip__value ${appState.connected ? "status-green" : "status-red"}`;

    const calStatus = byId("calStatus");
    calStatus.textContent = appState.calibrated ? "Calibrated" : "Not calibrated";
    calStatus.className = `status-chip__value ${appState.calibrated ? "status-green" : "status-yellow"}`;

    const runStatus = byId("runStatus");
    runStatus.textContent = appState.status;
    runStatus.className = `status-chip__value ${appState.running ? "status-blue" : "status-slate"}`;

    byId("progressStatus").textContent = `${appState.progressDone} / ${appState.progressTotal}`;
    const pct = appState.progressTotal ? Math.round((appState.progressDone / appState.progressTotal) * 100) : 0;
    byId("progressFill").style.width = `${pct}%`;
    byId("runProgressFill").style.width = `${pct}%`;
    byId("progressPercent").textContent = `${pct}%`;

    if (state.defaults) {
      byId("servoDefaultsInfo").textContent =
        `Server defaults: pen up S${state.defaults.pen_up_s}, pen down S${state.defaults.pen_down_s}, PID ${state.server_pid}`;
    }

    renderStateBadges();
    renderProgressTiming();
    updateActionStates();
    updateStepper();
  } catch (_) {
    // Keep previous state on poll failures.
  }
}

function currentImageFile() {
  return byId("imageFile").files[0] || null;
}

function showOriginalPreview(file) {
  const host = byId("originalPreview");
  if (!host) return;
  if (!file) {
    host.innerHTML = '<span class="small">No image loaded</span>';
    return;
  }
  const url = URL.createObjectURL(file);
  host.innerHTML = `<img src="${url}" alt="Original upload preview" />`;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function normalizeBounds(bounds) {
  if (!bounds) return null;
  const minX = Number(bounds.minX);
  const minY = Number(bounds.minY);
  const maxX = Number(bounds.maxX);
  const maxY = Number(bounds.maxY);
  if (![minX, minY, maxX, maxY].every(Number.isFinite)) return null;

  const epsilon = 0.0001;
  const width = Math.max(epsilon, maxX - minX);
  const height = Math.max(epsilon, maxY - minY);
  return {
    minX,
    minY,
    maxX: minX + width,
    maxY: minY + height,
  };
}

function expandBounds(bounds, padding) {
  const normalized = normalizeBounds(bounds);
  if (!normalized) return null;
  return {
    minX: normalized.minX - padding,
    minY: normalized.minY - padding,
    maxX: normalized.maxX + padding,
    maxY: normalized.maxY + padding,
  };
}

function computePathBounds(paths) {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;

  for (const entry of paths || []) {
    for (const point of entry.points || []) {
      if (!Number.isFinite(point.x) || !Number.isFinite(point.y)) continue;
      if (point.x < minX) minX = point.x;
      if (point.y < minY) minY = point.y;
      if (point.x > maxX) maxX = point.x;
      if (point.y > maxY) maxY = point.y;
    }
  }

  if (!Number.isFinite(minX)) return { ...FLAT_WORLD_LIMITS };
  return expandBounds({ minX, minY, maxX, maxY }, 1.25);
}

function formatZoomPercent(scale) {
  if (!Number.isFinite(scale) || scale <= 0) return "100%";
  return `${Math.round(scale * 100)}%`;
}

function updatePreviewZoomLabel(id, scale) {
  const label = byId(id);
  if (label) label.textContent = formatZoomPercent(scale);
}

class PanZoomCanvas {
  constructor(canvas, options = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.options = {
      minScale: options.minScale ?? 0.1,
      maxScale: options.maxScale ?? 40,
      zoomStep: options.zoomStep ?? 1.2,
      onChange: options.onChange ?? (() => {}),
    };
    this.contentBounds = null;
    this.drawCallback = null;
    this.scale = 1;
    this.offsetX = 0;
    this.offsetY = 0;
    this.cssWidth = 1;
    this.cssHeight = 1;
    this.dpr = window.devicePixelRatio || 1;
    this.defaultView = null;
    this.pointerId = null;
    this.isDragging = false;
    this.lastPointer = null;
    this.rafHandle = 0;

    this.handleWheel = this.handleWheel.bind(this);
    this.handlePointerDown = this.handlePointerDown.bind(this);
    this.handlePointerMove = this.handlePointerMove.bind(this);
    this.handlePointerUp = this.handlePointerUp.bind(this);
    this.handleDoubleClick = this.handleDoubleClick.bind(this);

    this.canvas.addEventListener("wheel", this.handleWheel, { passive: false });
    this.canvas.addEventListener("pointerdown", this.handlePointerDown);
    this.canvas.addEventListener("pointermove", this.handlePointerMove);
    this.canvas.addEventListener("pointerup", this.handlePointerUp);
    this.canvas.addEventListener("pointercancel", this.handlePointerUp);
    this.canvas.addEventListener("lostpointercapture", this.handlePointerUp);
    this.canvas.addEventListener("dblclick", this.handleDoubleClick);

    this.resize(false);
  }

  getViewState() {
    return {
      scale: this.scale,
      offsetX: this.offsetX,
      offsetY: this.offsetY,
      width: this.cssWidth,
      height: this.cssHeight,
      dpr: this.dpr,
      contentBounds: this.contentBounds,
    };
  }

  getEventPoint(event) {
    const rect = this.canvas.getBoundingClientRect();
    return {
      x: event.clientX - rect.left,
      y: event.clientY - rect.top,
    };
  }

  resize(preserveCenter = true) {
    const previousCenter = preserveCenter ? this.screenToWorld(this.cssWidth / 2, this.cssHeight / 2) : null;
    const rect = this.canvas.getBoundingClientRect();
    this.cssWidth = Math.max(300, Math.round(rect.width || 300));
    this.cssHeight = Math.max(300, Math.round(rect.height || 300));
    this.dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.max(1, Math.round(this.cssWidth * this.dpr));
    this.canvas.height = Math.max(1, Math.round(this.cssHeight * this.dpr));

    if (previousCenter && Number.isFinite(previousCenter.x) && Number.isFinite(previousCenter.y)) {
      this.offsetX = (this.cssWidth / 2) - (previousCenter.x * this.scale);
      this.offsetY = (this.cssHeight / 2) - (previousCenter.y * this.scale);
    }

    this.requestRender();
    this.options.onChange(this.getViewState());
  }

  setContentBounds(bounds) {
    this.contentBounds = normalizeBounds(bounds);
  }

  setTransform(scale, offsetX, offsetY) {
    this.scale = clamp(scale, this.options.minScale, this.options.maxScale);
    this.offsetX = Number.isFinite(offsetX) ? offsetX : this.offsetX;
    this.offsetY = Number.isFinite(offsetY) ? offsetY : this.offsetY;
    this.requestRender();
    this.options.onChange(this.getViewState());
  }

  fitToView(padding = 24) {
    if (!this.contentBounds) {
      this.setTransform(1, this.cssWidth / 2, this.cssHeight / 2);
      this.defaultView = { scale: this.scale, offsetX: this.offsetX, offsetY: this.offsetY };
      return;
    }

    const contentWidth = Math.max(0.0001, this.contentBounds.maxX - this.contentBounds.minX);
    const contentHeight = Math.max(0.0001, this.contentBounds.maxY - this.contentBounds.minY);
    const availableWidth = Math.max(1, this.cssWidth - (padding * 2));
    const availableHeight = Math.max(1, this.cssHeight - (padding * 2));
    const scale = clamp(
      Math.min(availableWidth / contentWidth, availableHeight / contentHeight),
      this.options.minScale,
      this.options.maxScale,
    );
    const contentCenterX = (this.contentBounds.minX + this.contentBounds.maxX) / 2;
    const contentCenterY = (this.contentBounds.minY + this.contentBounds.maxY) / 2;

    this.setTransform(
      scale,
      (this.cssWidth / 2) - (contentCenterX * scale),
      (this.cssHeight / 2) - (contentCenterY * scale),
    );
    this.defaultView = { scale: this.scale, offsetX: this.offsetX, offsetY: this.offsetY };
  }

  resetView() {
    if (!this.defaultView) {
      this.fitToView();
      return;
    }
    this.setTransform(this.defaultView.scale, this.defaultView.offsetX, this.defaultView.offsetY);
  }

  screenToWorld(x, y) {
    return {
      x: (x - this.offsetX) / this.scale,
      y: (y - this.offsetY) / this.scale,
    };
  }

  worldToScreen(x, y) {
    return {
      x: (x * this.scale) + this.offsetX,
      y: (y * this.scale) + this.offsetY,
    };
  }

  applyWorldTransform(ctx = this.ctx) {
    ctx.setTransform(
      this.dpr * this.scale,
      0,
      0,
      this.dpr * this.scale,
      this.dpr * this.offsetX,
      this.dpr * this.offsetY,
    );
  }

  zoomAt(factor, screenX, screenY) {
    const worldPoint = this.screenToWorld(screenX, screenY);
    const nextScale = clamp(this.scale * factor, this.options.minScale, this.options.maxScale);
    this.setTransform(
      nextScale,
      screenX - (worldPoint.x * nextScale),
      screenY - (worldPoint.y * nextScale),
    );
  }

  zoomTo(scale, screenX = this.cssWidth / 2, screenY = this.cssHeight / 2) {
    const worldPoint = this.screenToWorld(screenX, screenY);
    const nextScale = clamp(scale, this.options.minScale, this.options.maxScale);
    this.setTransform(
      nextScale,
      screenX - (worldPoint.x * nextScale),
      screenY - (worldPoint.y * nextScale),
    );
  }

  stepZoom(direction) {
    const factor = direction > 0 ? this.options.zoomStep : (1 / this.options.zoomStep);
    this.zoomAt(factor, this.cssWidth / 2, this.cssHeight / 2);
  }

  panBy(deltaX, deltaY) {
    this.setTransform(this.scale, this.offsetX + deltaX, this.offsetY + deltaY);
  }

  isLikelyTrackpadPan(event) {
    if (event.ctrlKey) return false;
    if (event.deltaMode !== WheelEvent.DOM_DELTA_PIXEL) return false;
    const precise = !Number.isInteger(event.deltaY) || Math.abs(event.deltaY) < 40;
    return Math.abs(event.deltaX) > 0 || precise;
  }

  handleWheel(event) {
    event.preventDefault();
    const point = this.getEventPoint(event);
    const deltaScale = event.deltaMode === WheelEvent.DOM_DELTA_LINE ? 16 : event.deltaMode === WheelEvent.DOM_DELTA_PAGE ? this.cssHeight : 1;
    const deltaX = event.deltaX * deltaScale;
    const deltaY = event.deltaY * deltaScale;

    if (this.isLikelyTrackpadPan(event)) {
      this.panBy(-deltaX, -deltaY);
      return;
    }

    this.zoomAt(Math.exp((-deltaY) * 0.0015), point.x, point.y);
  }

  handlePointerDown(event) {
    if (event.pointerType !== "touch" && event.button !== 0) return;
    event.preventDefault();
    this.pointerId = event.pointerId;
    this.isDragging = true;
    this.lastPointer = this.getEventPoint(event);
    this.canvas.setPointerCapture(event.pointerId);
    this.canvas.classList.add("is-dragging");
  }

  handlePointerMove(event) {
    if (!this.isDragging || event.pointerId !== this.pointerId) return;
    event.preventDefault();
    const point = this.getEventPoint(event);
    const deltaX = point.x - this.lastPointer.x;
    const deltaY = point.y - this.lastPointer.y;
    this.lastPointer = point;
    this.panBy(deltaX, deltaY);
  }

  handlePointerUp(event) {
    if (event.pointerId !== this.pointerId) return;
    this.isDragging = false;
    this.pointerId = null;
    this.lastPointer = null;
    this.canvas.classList.remove("is-dragging");
    if (this.canvas.hasPointerCapture(event.pointerId)) this.canvas.releasePointerCapture(event.pointerId);
  }

  handleDoubleClick(event) {
    event.preventDefault();
    this.resetView();
  }

  draw(drawCallback) {
    this.drawCallback = drawCallback;
    this.requestRender();
  }

  requestRender() {
    if (this.rafHandle) return;
    this.rafHandle = window.requestAnimationFrame(() => {
      this.rafHandle = 0;
      this.render();
    });
  }

  render() {
    const ctx = this.ctx;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.clearRect(0, 0, this.cssWidth, this.cssHeight);
    if (!this.drawCallback) return;
    this.drawCallback(ctx, {
      ...this.getViewState(),
      controller: this,
      screenToWorld: (x, y) => this.screenToWorld(x, y),
      worldToScreen: (x, y) => this.worldToScreen(x, y),
    });
  }
}

function getSelectedColorSet() {
  return new Set(appState.selectedColors);
}

function setSelectedColors(colors) {
  const changed = JSON.stringify(appState.selectedColors) !== JSON.stringify([...colors]);
  appState.selectedColors = [...colors];
  if (changed) invalidateGeneratedArtifacts();
  renderColorSwatches(appState.latestAnalysis?.colors || []);
  updateSelectedColorSummary();
  updateActionStates();
}

function updateSelectedColorSummary() {
  const host = byId("selectedColorSummary");
  if (!host) return;
  if (!appState.selectedColors.length) {
    host.textContent = "No colors selected.";
    return;
  }
  host.textContent = `Selected for printing: ${appState.selectedColors.join(", ")}`;
}

function guessInitialColors(colors) {
  if (!colors.length) return [];
  const dark = colors.filter((color) => color.luminance < 180);
  if (dark.length) {
    return [dark.sort((a, b) => a.luminance - b.luminance)[0].hex];
  }
  return [colors[0].hex];
}

function renderColorSwatches(colors) {
  const host = byId("colorSwatches");
  if (!host) return;
  if (!colors.length) {
    host.innerHTML = '<span class="small">Analyze an image to populate selectable colors.</span>';
    return;
  }
  const selected = getSelectedColorSet();
  host.innerHTML = colors.map((color) => {
    const active = selected.has(color.hex);
    const coverage = `${(color.coverage * 100).toFixed(1)}%`;
    return `
      <button
        type="button"
        class="swatch ${active ? "active" : ""}"
        data-color="${escapeHtml(color.hex)}"
      >
        <span class="swatch-chip" style="background:${escapeHtml(color.hex)}"></span>
        <span class="swatch-text">${escapeHtml(color.hex)}</span>
        <span class="swatch-meta">${coverage} · ${formatNumber(color.pixel_count)} px</span>
      </button>
    `;
  }).join("");

  host.querySelectorAll(".swatch").forEach((button) => {
    button.addEventListener("click", () => {
      toggleColorSelection(button.dataset.color);
    });
  });
}

function toggleColorSelection(hex) {
  const selected = getSelectedColorSet();
  if (selected.has(hex)) selected.delete(hex);
  else selected.add(hex);
  setSelectedColors([...selected]);
}

function setGenerateButtonBusy(isBusy) {
  const button = byId("generateGcodeButton");
  if (!button) return;
  button.disabled = isBusy;
  button.textContent = isBusy ? "Generating..." : "Generate G-code";
}

function renderSummary(summary) {
  const host = byId("jobSummary");
  if (!host) return;
  if (!summary) {
    host.innerHTML = '<div class="summary-empty">Generate G-code to see the job summary.</div>';
    return;
  }
  const entries = [
    ["Image size", summary.image_size],
    ["Selected colors", Array.isArray(summary.selected_colors) ? summary.selected_colors.join(", ") : "-"],
    ["Mask pixels", formatNumber(summary.mask_pixel_count)],
    ["Components", formatNumber(summary.component_count)],
    ["Walls", formatNumber(summary.wall_path_count)],
    ["Infill paths", formatNumber(summary.infill_path_count)],
    ["Detail traces", formatNumber(summary.detail_trace_path_count)],
    ["G-code lines", formatNumber(summary.gcode_line_count)],
    ["Plotted points", formatNumber(summary.point_count)],
    ["Estimated runtime", formatDuration(summary.estimated_runtime_seconds)],
  ];
  host.innerHTML = entries.map(([label, value]) => `
    <div class="summary-item">
      <span class="summary-item__label">${escapeHtml(label)}</span>
      <span class="summary-item__value">${escapeHtml(value || "-")}</span>
    </div>
  `).join("");
}

function invalidateGeneratedArtifacts() {
  if (!appState.latestGcode.length && !appState.latestSummary) return;
  appState.latestGcode = [];
  appState.latestPreview = [];
  appState.latestSummary = null;
  byId("gcodeBox").value = "";
  drawFlatPreview([]);
  drawBallPreview([]);
  renderSummary(null);
  renderProgressTiming();
  updateActionStates();
  updateStepper();
}

function renderStateBadges() {
  const connectBadge = byId("connectBadge");
  connectBadge.textContent = appState.connected ? "Connected" : "Offline";
  connectBadge.className = `badge ${appState.connected ? "badge-green" : "badge-red"}`;

  const calibrationBadge = byId("calibrationBadge");
  calibrationBadge.textContent = appState.calibrated ? "Calibrated" : "Required";
  calibrationBadge.className = `badge ${appState.calibrated ? "badge-green" : "badge-amber"}`;

  const hasImage = Boolean(currentImageFile());
  const imageBadge = byId("imageBadge");
  imageBadge.textContent = hasImage ? "Image loaded" : "Awaiting image";
  imageBadge.className = `badge ${hasImage ? "badge-blue" : "badge-slate"}`;

  const runBadge = byId("runBadge");
  const runReady = canRunJob();
  if (appState.running) {
    runBadge.textContent = appState.paused ? "Paused" : "Running";
    runBadge.className = "badge badge-blue";
  } else {
    runBadge.textContent = runReady ? "Ready" : "Locked";
    runBadge.className = `badge ${runReady ? "badge-green" : "badge-slate"}`;
  }

  const calibrationMessage = byId("calibrationMessage");
  calibrationMessage.textContent = appState.calibrated
    ? "Calibrated: current position is X0/Y0."
    : "Place the pen at the exact center of the ball before setting origin.";
}

function getActiveElapsedSeconds() {
  if (!appState.runStartedAt) return null;
  const nowSeconds = Date.now() / 1000;
  const currentPauseSeconds = appState.paused && appState.pauseStartedAt
    ? Math.max(0, nowSeconds - appState.pauseStartedAt)
    : 0;
  return Math.max(0, nowSeconds - appState.runStartedAt - appState.pausedDurationSeconds - currentPauseSeconds);
}

function renderProgressTiming() {
  const generatedEstimate = appState.latestSummary?.estimated_runtime_seconds ?? null;
  const elapsedSeconds = getActiveElapsedSeconds();
  let estimatedTotalSeconds = generatedEstimate;
  let remainingSeconds = null;

  if (elapsedSeconds != null && appState.progressDone > 0 && appState.progressTotal >= appState.progressDone) {
    estimatedTotalSeconds = elapsedSeconds / (appState.progressDone / Math.max(appState.progressTotal, 1));
    remainingSeconds = Math.max(0, estimatedTotalSeconds - elapsedSeconds);
  } else if (elapsedSeconds != null && generatedEstimate != null) {
    remainingSeconds = Math.max(0, generatedEstimate - elapsedSeconds);
  }

  byId("runElapsed").textContent = formatDuration(elapsedSeconds);
  byId("runRemaining").textContent = formatDuration(remainingSeconds);
  byId("runEstimatedTotal").textContent = formatDuration(estimatedTotalSeconds);

  if (appState.running) {
    byId("headerEta").textContent = `ETA ${formatDuration(remainingSeconds)}`;
  } else if (generatedEstimate != null) {
    byId("headerEta").textContent = `Est ${formatDuration(generatedEstimate)}`;
  } else {
    byId("headerEta").textContent = "ETA --:--";
  }
}

function canRunJob() {
  return appState.connected && appState.calibrated && appState.latestGcode.length > 0;
}

function updateActionStates() {
  const hasImage = Boolean(currentImageFile());
  const hasColors = appState.selectedColors.length > 0;
  const hasGcode = appState.latestGcode.length > 0;
  const machineBusy = appState.running || appState.generateRequestInFlight || appState.calibrationRequestInFlight;

  byId("analyzeButton").disabled = !hasImage || appState.generateRequestInFlight;
  byId("generateGcodeButton").disabled = !hasImage || !hasColors || appState.generateRequestInFlight;
  byId("runButton").disabled = !canRunJob() || machineBusy;
  byId("pauseButton").disabled = !appState.running || appState.paused;
  byId("resumeButton").disabled = !appState.running || !appState.paused;
  byId("stopButton").disabled = !appState.connected;
  byId("headerStopButton").disabled = !appState.connected;
  byId("downloadGcodeButton").disabled = !hasGcode;
  byId("zeroAndCalibrateButton").disabled = !appState.connected || appState.running || appState.calibrationRequestInFlight;
  byId("clearCalibratedButton").disabled = !appState.calibrated || appState.running;
}

function updateStepper() {
  const statuses = {
    "step-connect": appState.connected,
    "step-calibrate": appState.calibrated,
    "step-prepare": Boolean(currentImageFile()) && appState.selectedColors.length > 0,
    "step-generate": appState.latestGcode.length > 0,
    "step-run": canRunJob(),
  };

  document.querySelectorAll(".stepper__item").forEach((item) => {
    const stepId = item.dataset.step;
    item.classList.remove("done", "locked");
    if (statuses[stepId]) item.classList.add("done");
    else if (stepId !== "step-connect") item.classList.add("locked");
  });
}

function formDataEntries(form) {
  return Array.from(form.entries()).map(([key, value]) => {
    if (value instanceof File) return `${key}=[File:${value.name || "unnamed"}]`;
    return `${key}=${value}`;
  });
}

function buildRasterForm(file) {
  const form = new FormData();
  form.append("image", file);
  form.append("selected_colors", JSON.stringify(appState.selectedColors));
  form.append("draw_feed", getNum("drawFeed"));
  form.append("travel_feed", getNum("travelFeed"));
  form.append("sample_step_deg", getNum("sampleStepDeg"));
  form.append("margin_percent", getNum("marginPercent"));
  form.append("fit_mode", byId("fitMode").value);
  form.append("invert_y", getBool("invertY") ? "1" : "0");
  form.append("include_comments", getBool("includeComments") ? "1" : "0");
  form.append("debug_pipeline", getBool("debugPipeline") ? "1" : "0");
  form.append("placement_scale", getNum("placementScale"));
  form.append("placement_offset_x", getNum("placementOffsetX"));
  form.append("placement_offset_y", getNum("placementOffsetY"));
  form.append("rotation_deg", getNum("rotationDeg"));
  form.append("line_thickness_mm", getNum("lineThicknessMm"));
  form.append("wall_count", getInt("wallCount"));
  form.append("infill_pattern", byId("infillPattern").value);
  form.append("infill_density", getNum("infillDensity"));
  if (getValue("infillSpacingMm") !== "") form.append("infill_spacing_mm", getNum("infillSpacingMm"));
  form.append("infill_angle_deg", getNum("infillAngleDeg"));
  form.append("outline_after_fill", getBool("outlineAfterFill") ? "1" : "0");
  if (getValue("minFillAreaMm2") !== "") form.append("min_fill_area_mm2", getNum("minFillAreaMm2"));
  if (getValue("minFillWidthMm") !== "") form.append("min_fill_width_mm", getNum("minFillWidthMm"));
  form.append("simplify_tolerance_mm", getNum("simplifyToleranceMm"));
  form.append("remove_duplicate_paths", getBool("removeDuplicatePaths") ? "1" : "0");
  form.append("allow_pen_down_infill_connectors", getBool("allowPenDownInfillConnectors") ? "1" : "0");
  form.append("small_shape_mode", byId("smallShapeMode").value);
  form.append("thin_detail_mode", getBool("thinDetailMode") ? "1" : "0");
  form.append("thin_detail_min_area_mm2", getNum("thinDetailMinAreaMm2"));
  form.append("thin_detail_simplify_mm", getNum("thinDetailSimplifyMm"));
  form.append("thin_detail_overlap", getBool("thinDetailOverlap") ? "1" : "0");
  form.append("min_segment_length_mm", getNum("minSegmentLengthMm"));
  form.append("travel_optimization", byId("travelOptimization").value);
  form.append("color_tolerance", getInt("colorTolerance"));
  form.append("min_component_area_px", getInt("minComponentAreaPx"));
  form.append("mask_open_radius_px", getInt("maskOpenRadiusPx"));
  form.append("mask_close_radius_px", getInt("maskCloseRadiusPx"));
  form.append("min_region_area_px", getNum("minRegionAreaPx"));
  form.append("region_simplify_px", getNum("regionSimplifyPx"));
  form.append("pen_up_s", getInt("penUpS"));
  form.append("pen_down_s", getInt("penDownS"));
  form.append("servo_ramp_enabled", getBool("servoRampEnabled") ? "1" : "0");
  form.append("servo_ramp_step", getInt("servoRampStep"));
  form.append("servo_ramp_delay_ms", getNum("servoRampDelayMs"));
  form.append("pen_up_dwell_ms", getNum("penUpDwellMs"));
  form.append("pen_down_dwell_ms", getNum("penDownDwellMs"));
  return form;
}

async function analyzeRasterImage() {
  const file = currentImageFile();
  if (!file) {
    appendLog("ERROR: Choose a PNG or JPG file first.");
    showToast("Choose an image first.", "error");
    return;
  }

  showOriginalPreview(file);
  const form = new FormData();
  form.append("image", file);
  form.append("simplify_colors", getBool("simplifyColors") ? "1" : "0");
  form.append("max_colors", getInt("maxColors"));

  try {
    const res = await fetch("/analyze-image", { method: "POST", body: form });
    const json = await res.json();
    if (!json.ok) {
      appendLog(`ANALYZE ERROR: ${json.error}`);
      showToast(json.error || "Analyze failed", "error");
      return;
    }

    appState.latestAnalysis = json.analysis;
    if (!appState.selectedColors.length) {
      setSelectedColors(guessInitialColors(appState.latestAnalysis.colors || []));
    } else {
      renderColorSwatches(appState.latestAnalysis.colors || []);
      updateSelectedColorSummary();
    }
    appendLog(`Detected ${appState.latestAnalysis.colors.length} major colors in ${appState.latestAnalysis.width}x${appState.latestAnalysis.height} image.`);
    showToast("Colors analyzed.", "success");
    updateStepper();
  } catch (err) {
    appendLog(`ANALYZE FETCH ERROR: ${err}`);
    showToast(String(err), "error");
  }
}

async function generateRasterGcode() {
  const file = currentImageFile();
  if (appState.generateRequestInFlight) {
    appendLog("Generate request already running.");
    return;
  }
  if (!file) {
    appendLog("Choose an image first.");
    showToast("Choose an image first.", "error");
    return;
  }
  if (!appState.selectedColors.length) {
    appendLog("Select at least one color to print.");
    showToast("Select at least one color.", "error");
    return;
  }

  const form = buildRasterForm(file);
  appState.generateRequestInFlight = true;
  setGenerateButtonBusy(true);
  updateActionStates();
  appendLog(`POST /generate-image-gcode`);
  appendLog(`Generate form fields: ${formDataEntries(form).join(", ")}`);

  try {
    const res = await fetch("/generate-image-gcode", {
      method: "POST",
      body: form,
    });
    const { json, rawText } = await readJsonResponse(res);
    if (!json) {
      appendLog(`GENERATE ERROR: Non-JSON response (${res.status}).`);
      if (rawText) appendLog(rawText.slice(0, 1000));
      showToast("Generate failed.", "error");
      return;
    }
    if (!json.ok) {
      appendLog(`GENERATE ERROR: ${json.error || `HTTP ${res.status}`}`);
      if (json.debug) appendLog(`GENERATE DEBUG: ${JSON.stringify(json.debug)}`);
      showToast(json.error || "Generate failed.", "error");
      return;
    }

    appState.latestGcode = json.gcode || [];
    appState.latestPreview = json.preview || [];
    appState.latestMask = json.mask || null;
    appState.latestRegions = json.regions || null;
    appState.latestMaskPreview = json.mask_preview || appState.latestMask?.mask_preview_url || appState.latestRegions?.boundary_preview_url || null;
    appState.latestSummary = json.summary || null;

    byId("gcodeBox").value = appState.latestGcode.join("\n");
    drawMaskPreview(appState.latestMaskPreview);
    drawFlatPreview(appState.latestPreview);
    drawBallPreview(appState.latestPreview);
    renderSummary(appState.latestSummary);

    appendLog(`Generated ${appState.latestGcode.length} G-code lines from ${json.toolpath_count} toolpaths / ${json.point_count} plotted points.`);
    appendLog(`Selected colors: ${json.selected_colors.join(", ")}. Regions: ${appState.latestRegions?.region_count || 0}, holes: ${appState.latestRegions?.hole_count || 0}.`);
    appendLog(`Mask printable pixels: ${appState.latestMask?.printable_pixel_count || 0}. Area extracted from mask: ${(appState.latestRegions?.printable_area_px || 0).toFixed(1)} px^2.`);
    showToast("Toolpaths and G-code generated.", "success");
    openUtilityTab("summaryTab");
    await refreshState();
  } catch (err) {
    appendLog(`GENERATE FETCH ERROR: ${err}`);
    showToast(String(err), "error");
  } finally {
    appState.generateRequestInFlight = false;
    setGenerateButtonBusy(false);
    updateActionStates();
  }
}

function setupCanvas(canvasId) {
  const canvas = byId(canvasId);
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(300, Math.floor(rect.width * window.devicePixelRatio));
  canvas.height = Math.max(300, Math.floor(rect.height * window.devicePixelRatio));
  return canvas.getContext("2d");
}

function previewStyle(kind, depth = 1) {
  const alpha = Math.max(0.18, Math.min(1, depth));
  if (kind === "outline") return { stroke: `rgba(245, 158, 11, ${alpha})`, width: 2.2, dash: [] };
  if (kind === "fill-wall") return { stroke: `rgba(245, 158, 11, ${alpha})`, width: 2.6, dash: [] };
  if (kind === "fill-infill") return { stroke: `rgba(45, 212, 191, ${alpha})`, width: 1.7, dash: [] };
  if (kind === "crossed-contour-infill") return { stroke: `rgba(20, 184, 166, ${alpha})`, width: 1.6, dash: [3, 3] };
  if (kind === "repair-patch-fill") return { stroke: `rgba(251, 113, 133, ${alpha})`, width: 2.0, dash: [] };
  if (kind === "repair-patch-shape") return { stroke: `rgba(244, 63, 94, ${alpha})`, width: 1.5, dash: [6, 4] };
  if (kind === "junction-centerline") return { stroke: `rgba(250, 204, 21, ${alpha})`, width: 1.7, dash: [] };
  if (kind === "gap-repair-stroke") return { stroke: `rgba(251, 113, 133, ${alpha})`, width: 2.0, dash: [] };
  if (kind === "gap-repair-dab") return { stroke: `rgba(239, 68, 68, ${alpha})`, width: 2.1, dash: [2, 2] };
  if (kind.startsWith("gap-residual")) return { stroke: `rgba(255, 255, 255, ${alpha})`, width: 1.4, dash: [5, 4] };
  if (kind.startsWith("gap-repair-rejected")) return { stroke: `rgba(248, 113, 113, ${alpha})`, width: 1.3, dash: [7, 5] };
  if (kind === "detail-trace") return { stroke: `rgba(232, 121, 249, ${alpha})`, width: 2.1, dash: [] };
  if (kind === "debug-valid-connector") return { stroke: `rgba(34, 197, 94, ${alpha})`, width: 1.6, dash: [] };
  if (kind === "debug-rejected-connector") return { stroke: `rgba(239, 68, 68, ${alpha})`, width: 1.4, dash: [8, 6] };
  return { stroke: `rgba(148, 163, 184, ${alpha * 0.9})`, width: 1.1, dash: [6, 6] };
}

function drawFlatPreview(paths) {
  const ctx = setupCanvas("flatPreview");
  const w = ctx.canvas.width;
  const h = ctx.canvas.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#020617";
  ctx.fillRect(0, 0, w, h);

  const pad = 28 * window.devicePixelRatio;
  const plotW = w - pad * 2;
  const plotH = h - pad * 2;
  const scale = Math.min(plotW / 360, plotH / 90);
  const drawW = 360 * scale;
  const drawH = 90 * scale;
  const left = (w - drawW) / 2;
  const top = (h - drawH) / 2;
  const sx = (x) => left + (x + 180) * scale;
  const sy = (y) => top + (45 - y) * scale;

  ctx.strokeStyle = "#334155";
  ctx.lineWidth = 1 * window.devicePixelRatio;
  ctx.strokeRect(left, top, drawW, drawH);

  ctx.fillStyle = "rgba(59, 130, 246, 0.28)";
  ctx.fillRect(sx(0) - 1 * window.devicePixelRatio, top, 2 * window.devicePixelRatio, drawH);
  ctx.fillRect(left, sy(0) - 1 * window.devicePixelRatio, drawW, 2 * window.devicePixelRatio);

  ctx.fillStyle = "#94a3b8";
  ctx.font = `${11 * window.devicePixelRatio}px Segoe UI`;
  ctx.fillText("X -180°", left, h - 9 * window.devicePixelRatio);
  ctx.fillText("X 0°", left + (180 * scale) - 13 * window.devicePixelRatio, h - 9 * window.devicePixelRatio);
  ctx.fillText("X +180°", w - left - 54 * window.devicePixelRatio, h - 9 * window.devicePixelRatio);

  for (const entry of paths) {
    const path = entry.points || [];
    if (!path.length) continue;
    const style = previewStyle(entry.kind || "outline");
    ctx.beginPath();
    ctx.moveTo(sx(path[0].x), sy(path[0].y));
    for (let index = 1; index < path.length; index += 1) {
      ctx.lineTo(sx(path[index].x), sy(path[index].y));
    }
    ctx.setLineDash((style.dash || []).map((value) => value * window.devicePixelRatio));
    ctx.lineWidth = style.width * window.devicePixelRatio;
    ctx.strokeStyle = style.stroke;
    ctx.stroke();
  }
  ctx.setLineDash([]);
}

function drawPreviewEmptyState(ctx, width, height, message) {
  ctx.save();
  ctx.fillStyle = "#020617";
  ctx.fillRect(0, 0, width, height);
  ctx.fillStyle = "rgba(148, 163, 184, 0.92)";
  ctx.font = '500 14px "Segoe UI"';
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(message, width / 2, height / 2);
  ctx.restore();
}

function chooseGridStep(scale) {
  const targetScreenSpacing = 72;
  const candidates = [1, 2, 5, 10, 15, 30, 45, 90];
  for (const step of candidates) {
    if ((step * scale) >= targetScreenSpacing) return step;
  }
  return 90;
}

function drawFlatWorldGrid(ctx, view, controller) {
  const visibleTopLeft = controller.screenToWorld(0, 0);
  const visibleBottomRight = controller.screenToWorld(view.width, view.height);
  const visibleBounds = {
    minX: Math.min(visibleTopLeft.x, visibleBottomRight.x),
    maxX: Math.max(visibleTopLeft.x, visibleBottomRight.x),
    minY: Math.min(visibleTopLeft.y, visibleBottomRight.y),
    maxY: Math.max(visibleTopLeft.y, visibleBottomRight.y),
  };
  const gridStep = chooseGridStep(view.scale);

  ctx.save();
  controller.applyWorldTransform(ctx);

  ctx.strokeStyle = "rgba(51, 65, 85, 0.72)";
  ctx.lineWidth = 1 / view.scale;
  for (let x = Math.floor(visibleBounds.minX / gridStep) * gridStep; x <= visibleBounds.maxX; x += gridStep) {
    ctx.beginPath();
    ctx.moveTo(x, visibleBounds.minY);
    ctx.lineTo(x, visibleBounds.maxY);
    ctx.stroke();
  }
  for (let y = Math.floor(visibleBounds.minY / gridStep) * gridStep; y <= visibleBounds.maxY; y += gridStep) {
    ctx.beginPath();
    ctx.moveTo(visibleBounds.minX, y);
    ctx.lineTo(visibleBounds.maxX, y);
    ctx.stroke();
  }

  ctx.strokeStyle = "rgba(148, 163, 184, 0.8)";
  ctx.lineWidth = 1.3 / view.scale;
  ctx.strokeRect(
    FLAT_WORLD_LIMITS.minX,
    FLAT_WORLD_LIMITS.minY,
    FLAT_WORLD_LIMITS.maxX - FLAT_WORLD_LIMITS.minX,
    FLAT_WORLD_LIMITS.maxY - FLAT_WORLD_LIMITS.minY,
  );

  ctx.strokeStyle = "rgba(59, 130, 246, 0.86)";
  ctx.lineWidth = 1.8 / view.scale;
  ctx.beginPath();
  ctx.moveTo(0, visibleBounds.minY);
  ctx.lineTo(0, visibleBounds.maxY);
  ctx.moveTo(visibleBounds.minX, 0);
  ctx.lineTo(visibleBounds.maxX, 0);
  ctx.stroke();
  ctx.restore();
}

function renderMaskPreview(ctx, view) {
  ctx.fillStyle = "#020617";
  ctx.fillRect(0, 0, view.width, view.height);

  if (!previewData.maskImage) {
    drawPreviewEmptyState(ctx, view.width, view.height, PREVIEW_EMPTY_TEXT.mask);
    return;
  }

  ctx.save();
  view.controller.applyWorldTransform(ctx);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(previewData.maskImage, 0, 0, previewData.maskImage.width, previewData.maskImage.height);
  ctx.strokeStyle = "rgba(148, 163, 184, 0.7)";
  ctx.lineWidth = 1 / view.scale;
  ctx.strokeRect(0, 0, previewData.maskImage.width, previewData.maskImage.height);
  ctx.restore();
}

function renderFlatPreview(ctx, view) {
  ctx.fillStyle = "#020617";
  ctx.fillRect(0, 0, view.width, view.height);
  drawFlatWorldGrid(ctx, view, view.controller);

  if (!previewData.flatPaths.length) {
    drawPreviewEmptyState(ctx, view.width, view.height, PREVIEW_EMPTY_TEXT.flat);
    return;
  }

  ctx.save();
  view.controller.applyWorldTransform(ctx);
  for (const entry of previewData.flatPaths) {
    const path = entry.points || [];
    if (!path.length) continue;
    const style = previewStyle(entry.kind || "outline");
    ctx.beginPath();
    ctx.moveTo(path[0].x, path[0].y);
    for (let index = 1; index < path.length; index += 1) {
      ctx.lineTo(path[index].x, path[index].y);
    }
    const screenStrokeWidth = clamp(style.width, FLAT_STROKE_SCREEN_MIN, FLAT_STROKE_SCREEN_MAX);
    ctx.setLineDash((style.dash || []).map((value) => value / view.scale));
    ctx.lineWidth = screenStrokeWidth / view.scale;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.strokeStyle = style.stroke;
    ctx.stroke();
  }
  ctx.restore();
  ctx.setLineDash([]);
}

function ensurePreviewViews() {
  if (!previewViews.mask) {
    previewViews.mask = new PanZoomCanvas(byId("maskPreviewCanvas"), {
      minScale: 0.1,
      maxScale: 48,
      onChange: (view) => updatePreviewZoomLabel("maskZoomLevel", view.scale),
    });
    previewViews.mask.draw(renderMaskPreview);
  }

  if (!previewViews.flat) {
    previewViews.flat = new PanZoomCanvas(byId("flatPreview"), {
      minScale: 0.2,
      maxScale: 60,
      onChange: (view) => updatePreviewZoomLabel("flatZoomLevel", view.scale),
    });
    previewViews.flat.draw(renderFlatPreview);
  }
}

let latestMaskPreviewToken = 0;

function drawMaskPreview(dataUrl, options = {}) {
  ensurePreviewViews();
  const { fit = true } = options;
  const controller = previewViews.mask;
  const token = ++latestMaskPreviewToken;

  if (!dataUrl) {
    previewData.maskImage = null;
    controller.setContentBounds(null);
    controller.setTransform(1, 0, 0);
    controller.defaultView = { scale: controller.scale, offsetX: controller.offsetX, offsetY: controller.offsetY };
    controller.draw(renderMaskPreview);
    return;
  }

  const image = new Image();
  image.decoding = "async";
  image.onload = () => {
    if (token !== latestMaskPreviewToken) return;
    previewData.maskImage = image;
    controller.setContentBounds({
      minX: 0,
      minY: 0,
      maxX: image.naturalWidth || image.width,
      maxY: image.naturalHeight || image.height,
    });
    if (fit) controller.fitToView();
    else controller.draw(renderMaskPreview);
  };
  image.onerror = () => {
    if (token !== latestMaskPreviewToken) return;
    previewData.maskImage = null;
    controller.draw(renderMaskPreview);
  };
  image.src = dataUrl;
}

function drawFlatPreview(paths, options = {}) {
  ensurePreviewViews();
  const { fit = true } = options;
  previewData.flatPaths = Array.isArray(paths) ? paths : [];
  previewViews.flat.setContentBounds(computePathBounds(previewData.flatPaths));
  if (fit) previewViews.flat.fitToView();
  else previewViews.flat.draw(renderFlatPreview);
}

function projectBallPoint(point, cx, cy, radius) {
  const lon = (point.x * Math.PI) / 180;
  const lat = (point.y * Math.PI) / 180;
  const x = Math.cos(lat) * Math.sin(lon);
  const y = Math.sin(lat);
  const z = Math.cos(lat) * Math.cos(lon);
  const perspective = 0.72 + (z * 0.28);
  return {
    x: cx + y * radius * perspective,
    y: cy + x * radius * perspective,
    z,
  };
}

function drawBallPreview(paths) {
  const ctx = setupCanvas("ballPreview");
  const w = ctx.canvas.width;
  const h = ctx.canvas.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#020617";
  ctx.fillRect(0, 0, w, h);

  const radius = Math.min(w, h) * 0.38;
  const cx = w / 2;
  const cy = h / 2;

  const sphere = ctx.createRadialGradient(cx - radius * 0.35, cy - radius * 0.35, radius * 0.1, cx, cy, radius);
  sphere.addColorStop(0, "#f8fafc");
  sphere.addColorStop(0.22, "#cbd5e1");
  sphere.addColorStop(0.65, "#475569");
  sphere.addColorStop(1, "#0f172a");
  ctx.fillStyle = sphere;
  ctx.beginPath();
  ctx.arc(cx, cy, radius, 0, Math.PI * 2);
  ctx.fill();

  ctx.strokeStyle = "#475569";
  ctx.lineWidth = 1 * window.devicePixelRatio;
  ctx.beginPath();
  ctx.arc(cx, cy, radius, 0, Math.PI * 2);
  ctx.stroke();

  const projectedPaths = [];
  for (const entry of paths) {
    const path = entry.points || [];
    if (!path.length) continue;
    projectedPaths.push({
      kind: entry.kind || "outline",
      points: path.map((point) => projectBallPoint(point, cx, cy, radius)),
    });
  }
  projectedPaths.sort((a, b) => ((a.points[0]?.z || 0) - (b.points[0]?.z || 0)));

  for (const entry of projectedPaths) {
    const projected = entry.points;
    ctx.beginPath();
    for (let index = 0; index < projected.length; index += 1) {
      const point = projected[index];
      if (index === 0) ctx.moveTo(point.x, point.y);
      else ctx.lineTo(point.x, point.y);
    }
    const depth = projected.reduce((sum, point) => sum + point.z, 0) / projected.length;
    const style = previewStyle(entry.kind, 0.35 + ((depth + 1) * 0.325));
    ctx.setLineDash((style.dash || []).map((value) => value * window.devicePixelRatio));
    ctx.lineWidth = style.width * window.devicePixelRatio;
    ctx.strokeStyle = style.stroke;
    ctx.stroke();
  }
  ctx.setLineDash([]);
}

function downloadGcode() {
  const text = byId("gcodeBox").value;
  if (!text.trim()) {
    appendLog("ERROR: No G-code generated.");
    showToast("No G-code generated.", "error");
    return;
  }
  const blob = new Blob([text], { type: "text/plain" });
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = "golfball_plotter_output.gcode";
  link.click();
  URL.revokeObjectURL(link.href);
}

function openUtilityTab(id) {
  document.querySelectorAll(".utility-tab").forEach((button) => {
    button.classList.toggle("active", button.dataset.tab === id);
  });
  document.querySelectorAll(".utility-pane").forEach((pane) => {
    pane.classList.toggle("active", pane.id === id);
  });
}

async function connectMachine() {
  await api("/connect", {}, { successMessage: "Controller connected." });
}

async function applyMachineConfig() {
  await api("/apply-config", {
    x_max_feed: getNum("xMaxFeed"),
    y_max_feed: getNum("yMaxFeed"),
    x_acceleration: getNum("xAcceleration"),
    y_acceleration: getNum("yAcceleration"),
  }, { successMessage: "Machine settings applied." });
}

function sendCommand(command) {
  return api("/command", { command });
}

function sendRaw() {
  return sendCommand(byId("rawCommand").value);
}

function softReset() {
  return api("/reset", {}, { successMessage: "Soft reset sent." });
}

function penUp() {
  return api("/pen-up", appendServoSettings({ s: getInt("penUpS") }), { successMessage: "Pen moved up." });
}

function penDown() {
  return api("/pen-down", appendServoSettings({ s: getInt("penDownS") }), { successMessage: "Pen moved down." });
}

function penTest() {
  return api("/pen-test", appendServoSettings({
    up_s: getInt("penUpS"),
    down_s: getInt("penDownS"),
  }));
}

function servoOff() {
  return api("/servo-off", {}, { successMessage: "Servo disabled." });
}

function jog(axis, degrees) {
  return api("/jog", { axis, degrees, feed: getNum("travelFeed") });
}

function goHome() {
  return api("/go-home", appendServoSettings({
    pen_up_s: getInt("penUpS"),
    travel_feed: getNum("travelFeed"),
  }));
}

function clearCalibrated() {
  appState.latestGcode = appState.latestGcode;
  return api("/clear-calibrated", {}, { successMessage: "Calibration cleared." });
}

function runGcode() {
  return api("/run-gcode", {}, { successMessage: "Job started." });
}

function pauseRun() {
  return api("/pause", {}, { successMessage: "Pause requested." });
}

function resumeRun() {
  return api("/resume", {}, { successMessage: "Resume requested." });
}

function stopRun() {
  return api("/stop", {}, { successMessage: "Stop requested." });
}

async function zeroAndMarkCalibrated() {
  if (!appState.connected) {
    showToast("Connect the machine first.", "error");
    return;
  }
  appState.calibrationRequestInFlight = true;
  updateActionStates();
  const result = await api("/zero-and-mark-calibrated", {}, {
    successMessage: "Calibrated: current position is X0/Y0.",
  });
  appState.calibrationRequestInFlight = false;
  updateActionStates();
  if (result.ok) {
    showToast("Run is now available if G-code has been generated.", "info");
  }
}

function handleCalibrationDialog() {
  const dialog = byId("calibrationConfirmDialog");
  if (!dialog) return;
  if (typeof dialog.showModal === "function") dialog.showModal();
  else zeroAndMarkCalibrated();
}

function fitAll2DPreviews() {
  ensurePreviewViews();
  previewViews.mask?.fitToView();
  previewViews.flat?.fitToView();
}

function resetAll2DPreviews() {
  ensurePreviewViews();
  previewViews.mask?.resetView();
  previewViews.flat?.resetView();
}

function handlePreviewControlAction(target, action) {
  ensurePreviewViews();
  const controller = previewViews[target];
  if (!controller) return;

  if (action === "fit") controller.fitToView();
  else if (action === "reset") controller.resetView();
  else if (action === "zoom-in") controller.stepZoom(1);
  else if (action === "zoom-out") controller.stepZoom(-1);
  else if (action === "zoom-100") controller.zoomTo(1);
}

function bindEvents() {
  ensurePreviewViews();
  byId("connectButton")?.addEventListener("click", connectMachine);
  byId("applyMachineConfigButton")?.addEventListener("click", applyMachineConfig);
  byId("statusButton")?.addEventListener("click", () => sendCommand("?"));
  byId("settingsButton")?.addEventListener("click", () => sendCommand("$$"));
  byId("firmwareButton")?.addEventListener("click", () => sendCommand("$I"));
  byId("softResetButton")?.addEventListener("click", softReset);

  byId("penUpButton")?.addEventListener("click", penUp);
  byId("penDownButton")?.addEventListener("click", penDown);
  byId("penTestButton")?.addEventListener("click", penTest);
  byId("goHomeButton")?.addEventListener("click", goHome);
  byId("servoOffButton")?.addEventListener("click", servoOff);
  byId("resetServoDefaultsButton")?.addEventListener("click", resetServoUiDefaults);

  byId("jogUpButton")?.addEventListener("click", () => jog("Y", getNum("yJog")));
  byId("jogDownButton")?.addEventListener("click", () => jog("Y", -getNum("yJog")));
  byId("jogLeftButton")?.addEventListener("click", () => jog("X", -getNum("xJog")));
  byId("jogRightButton")?.addEventListener("click", () => jog("X", getNum("xJog")));

  byId("zeroAndCalibrateButton")?.addEventListener("click", handleCalibrationDialog);
  byId("clearCalibratedButton")?.addEventListener("click", clearCalibrated);

  byId("analyzeButton")?.addEventListener("click", analyzeRasterImage);
  byId("generateGcodeButton")?.addEventListener("click", (event) => {
    event.preventDefault();
    generateRasterGcode();
  });
  byId("fitPreviewButton")?.addEventListener("click", () => {
    fitAll2DPreviews();
  });
  byId("resetPreviewButton")?.addEventListener("click", () => {
    resetAll2DPreviews();
  });

  document.querySelectorAll("[data-preview-target][data-preview-action]").forEach((button) => {
    button.addEventListener("click", () => {
      handlePreviewControlAction(button.dataset.previewTarget, button.dataset.previewAction);
    });
  });

  byId("runButton")?.addEventListener("click", runGcode);
  byId("pauseButton")?.addEventListener("click", pauseRun);
  byId("resumeButton")?.addEventListener("click", resumeRun);
  byId("stopButton")?.addEventListener("click", stopRun);
  byId("headerStopButton")?.addEventListener("click", stopRun);

  byId("sendRawButton")?.addEventListener("click", sendRaw);
  byId("downloadGcodeButton")?.addEventListener("click", downloadGcode);

  byId("imageFile")?.addEventListener("change", () => {
    const file = currentImageFile();
    showOriginalPreview(file);
    invalidateGeneratedArtifacts();
    if (!file) {
      appState.latestAnalysis = null;
      appState.latestMask = null;
      appState.latestRegions = null;
      appState.latestMaskPreview = null;
      appState.latestSummary = null;
      setSelectedColors([]);
      drawMaskPreview(null);
      updateActionStates();
      updateStepper();
      return;
    }
    setSelectedColors([]);
    updateActionStates();
    updateStepper();
  });

  document.querySelectorAll(".utility-tab").forEach((button) => {
    button.addEventListener("click", () => openUtilityTab(button.dataset.tab));
  });

  [
    "drawFeed",
    "travelFeed",
    "placementScale",
    "placementOffsetX",
    "placementOffsetY",
    "rotationDeg",
    "lineThicknessMm",
    "wallCount",
    "infillPattern",
    "infillDensity",
    "infillSpacingMm",
    "infillAngleDeg",
    "outlineAfterFill",
    "minFillAreaMm2",
    "minFillWidthMm",
    "simplifyToleranceMm",
    "removeDuplicatePaths",
    "smallShapeMode",
    "thinDetailMode",
    "thinDetailMinAreaMm2",
    "thinDetailSimplifyMm",
    "thinDetailOverlap",
    "minSegmentLengthMm",
    "travelOptimization",
    "colorTolerance",
    "minComponentAreaPx",
    "maskOpenRadiusPx",
    "maskCloseRadiusPx",
    "minRegionAreaPx",
    "regionSimplifyPx",
    "sampleStepDeg",
    "includeComments",
    "invertY",
    "debugPipeline",
  ].forEach((id) => {
    byId(id)?.addEventListener("change", invalidateGeneratedArtifacts);
    byId(id)?.addEventListener("input", () => {
      if (["lineThicknessMm", "infillSpacingMm", "minFillWidthMm", "minFillAreaMm2"].includes(id)) {
        invalidateGeneratedArtifacts();
      }
    });
  });

  byId("utilityToggleButton")?.addEventListener("click", () => {
    byId("utilityDrawer").classList.toggle("open");
  });

  document.querySelectorAll(".stepper__item").forEach((button) => {
    button.addEventListener("click", () => {
      byId(button.dataset.step)?.scrollIntoView({ behavior: "smooth", block: "start" });
    });
  });

  const dialog = byId("calibrationConfirmDialog");
  const confirmButton = byId("confirmCalibrationButton");
  if (dialog && confirmButton) {
    confirmButton.addEventListener("click", (event) => {
      event.preventDefault();
      dialog.close("confirm");
      zeroAndMarkCalibrated();
    });
  }

  window.addEventListener("resize", () => {
    previewViews.mask?.resize(true);
    previewViews.flat?.resize(true);
    drawBallPreview(appState.latestPreview);
  });
}

setupDerivedPenFieldSync();
bindEvents();
syncDerivedPenFields(true);
showOriginalPreview(null);
drawMaskPreview(null);
renderSummary(null);
renderProgressTiming();
drawFlatPreview([]);
drawBallPreview([]);
openUtilityTab("summaryTab");
appendLog("Controller loaded. Connect, calibrate, analyze a PNG/JPG, select colors, generate G-code, then run.");
refreshState();
window.setInterval(refreshState, 750);
