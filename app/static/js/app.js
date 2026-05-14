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

function showMaskPreview(dataUrl) {
  const host = byId("maskPreview");
  if (!host) return;
  if (!dataUrl) {
    host.innerHTML = '<span class="small">No mask preview available</span>';
    return;
  }
  host.innerHTML = `<img src="${dataUrl}" alt="Selected color mask preview" />`;
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
    showMaskPreview(appState.latestMaskPreview);
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
  if (kind === "detail-trace") return { stroke: `rgba(232, 121, 249, ${alpha})`, width: 2.1, dash: [] };
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

function bindEvents() {
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
    drawFlatPreview(appState.latestPreview);
    drawBallPreview(appState.latestPreview);
  });
  byId("resetPreviewButton")?.addEventListener("click", () => {
    drawFlatPreview(appState.latestPreview);
    drawBallPreview(appState.latestPreview);
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
      showMaskPreview(null);
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
    drawFlatPreview(appState.latestPreview);
    drawBallPreview(appState.latestPreview);
  });
}

setupDerivedPenFieldSync();
bindEvents();
syncDerivedPenFields(true);
showOriginalPreview(null);
showMaskPreview(null);
renderSummary(null);
renderProgressTiming();
drawFlatPreview([]);
drawBallPreview([]);
openUtilityTab("summaryTab");
appendLog("Controller loaded. Connect, calibrate, analyze a PNG/JPG, select colors, generate G-code, then run.");
refreshState();
window.setInterval(refreshState, 750);
