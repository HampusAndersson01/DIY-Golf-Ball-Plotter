let latestGcode = [];
let latestPreview = [];
let latestAnalysis = null;
let latestMask = null;
let latestRegions = null;
let selectedColors = new Set();

function appendLog(text) {
  const log = document.getElementById("log");
  const now = new Date().toLocaleTimeString();
  log.textContent += `[${now}] ${text}\n`;
  log.scrollTop = log.scrollHeight;
}

function getNum(id) {
  return parseFloat(document.getElementById(id).value || "0");
}

function getInt(id) {
  return parseInt(document.getElementById(id).value || "0", 10);
}

function getBool(id) {
  return document.getElementById(id).checked;
}

function getValue(id) {
  return document.getElementById(id).value.trim();
}

const SERVER_DEFAULTS = window.SERVER_DEFAULTS || {
  penUpS: 575,
  penDownS: 700,
  penUpDwellMs: 30,
  penDownDwellMs: 60,
  servoRampEnabled: true,
  servoRampStep: 20,
  servoRampDelayMs: 10
};

const DERIVED_PEN_FIELDS = {
  infillSpacingMm: (penThickness) => penThickness,
  minFillWidthMm: (penThickness) => penThickness,
  minFillAreaMm2: (penThickness) => penThickness * penThickness
};

function roundTo(value, decimals) {
  const factor = 10 ** decimals;
  return Math.round(value * factor) / factor;
}

function syncDerivedPenFields(force = false) {
  const penThickness = Math.max(0, getNum("lineThicknessMm"));
  for (const [id, compute] of Object.entries(DERIVED_PEN_FIELDS)) {
    const input = document.getElementById(id);
    if (!input) continue;
    if (!force && input.dataset.userOverride === "1") continue;
    const decimals = id === "minFillAreaMm2" ? 3 : 2;
    input.value = String(roundTo(compute(penThickness), decimals));
    input.dataset.userOverride = "0";
  }

  const summary = document.getElementById("derivedPenSummary");
  if (summary) {
    const outlineInset = roundTo(penThickness / 2, 3);
    const infillSpacing = roundTo(DERIVED_PEN_FIELDS.infillSpacingMm(penThickness), 3);
    const minFillWidth = roundTo(DERIVED_PEN_FIELDS.minFillWidthMm(penThickness), 3);
    const minFillArea = roundTo(DERIVED_PEN_FIELDS.minFillAreaMm2(penThickness), 3);
    summary.textContent = `Auto defaults: outline inset ${outlineInset} mm, infill spacing ${infillSpacing} mm, min fill width ${minFillWidth} mm, min fill area ${minFillArea} mm^2.`;
  }
}

function setupDerivedPenFieldSync() {
  const penInput = document.getElementById("lineThicknessMm");
  if (penInput) {
    penInput.addEventListener("input", () => syncDerivedPenFields(false));
  }

  for (const id of Object.keys(DERIVED_PEN_FIELDS)) {
    const input = document.getElementById(id);
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
  document.getElementById("penUpS").value = SERVER_DEFAULTS.penUpS;
  document.getElementById("penDownS").value = SERVER_DEFAULTS.penDownS;
  document.getElementById("penUpDwellMs").value = SERVER_DEFAULTS.penUpDwellMs;
  document.getElementById("penDownDwellMs").value = SERVER_DEFAULTS.penDownDwellMs;
  document.getElementById("servoRampEnabled").checked = SERVER_DEFAULTS.servoRampEnabled;
  document.getElementById("servoRampStep").value = SERVER_DEFAULTS.servoRampStep;
  document.getElementById("servoRampDelayMs").value = SERVER_DEFAULTS.servoRampDelayMs;
  document.getElementById("rawCommand").value = `M3 S${SERVER_DEFAULTS.penUpS}`;
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

async function api(url, data = {}) {
  try {
    const res = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(data)
    });
    const json = await res.json();
    if (json.ok) {
      if (json.command) appendLog(`> ${json.command}`);
      if (json.response) appendLog(`< ${json.response}`);
    } else {
      appendLog(`ERROR: ${json.error}`);
    }
    await refreshState();
    return json;
  } catch (err) {
    appendLog(`FETCH ERROR: ${err}`);
    return { ok: false, error: String(err) };
  }
}

async function refreshState() {
  try {
    const res = await fetch("/state");
    const s = await res.json();
    document.getElementById("serialStatus").textContent = s.connected ? "Connected" : "Disconnected";
    document.getElementById("serialStatus").className = s.connected ? "value ok-text" : "value danger-text";
    document.getElementById("calStatus").textContent = s.calibrated ? "Ready" : "Not calibrated";
    document.getElementById("calStatus").className = s.calibrated ? "value ok-text" : "value warning";
    document.getElementById("runStatus").textContent = s.status || "-";
    document.getElementById("progressStatus").textContent = `${s.progress_done || 0} / ${s.progress_total || 0}`;
    if (s.defaults) {
      document.getElementById("servoDefaultsInfo").textContent =
        `Server defaults: pen up S${s.defaults.pen_up_s}, pen down S${s.defaults.pen_down_s}, PID ${s.server_pid}`;
    }
    const pct = s.progress_total ? Math.round((s.progress_done / s.progress_total) * 100) : 0;
    document.getElementById("progressFill").style.width = `${pct}%`;
  } catch (_) {}
}

function sendCommand(command) { return api("/command", { command }); }
function sendRaw() { return sendCommand(document.getElementById("rawCommand").value); }
function softReset() { return api("/reset"); }
function applyMachineConfig() {
  return api("/apply-config", {
    x_max_feed: getNum("xMaxFeed"),
    y_max_feed: getNum("yMaxFeed"),
    x_acceleration: getNum("xAcceleration"),
    y_acceleration: getNum("yAcceleration")
  });
}
function penUp() { return api("/pen-up", appendServoSettings({ s: getInt("penUpS") })); }
function penDown() { return api("/pen-down", appendServoSettings({ s: getInt("penDownS") })); }
function penTest() {
  return api("/pen-test", appendServoSettings({
    up_s: getInt("penUpS"),
    down_s: getInt("penDownS")
  }));
}
function servoOff() { return api("/servo-off"); }
function jogX(degrees) { return api("/jog", { axis: "X", degrees, feed: getNum("travelFeed") }); }
function jogY(degrees) { return api("/jog", { axis: "Y", degrees, feed: getNum("travelFeed") }); }
function zeroPosition() { return api("/zero-position"); }
function goHome() {
  return api("/go-home", appendServoSettings({
    pen_up_s: getInt("penUpS"),
    travel_feed: getNum("travelFeed")
  }));
}
function markCalibrated() { return api("/mark-calibrated"); }
function clearCalibrated() { return api("/clear-calibrated"); }
function runGcode() { return api("/run-gcode"); }
function pauseRun() { return api("/pause"); }
function resumeRun() { return api("/resume"); }
function stopRun() { return api("/stop"); }

function currentImageFile() {
  return document.getElementById("imageFile").files[0] || null;
}

function showOriginalPreview(file) {
  const host = document.getElementById("originalPreview");
  if (!file) {
    host.innerHTML = '<span class="small">No image loaded</span>';
    return;
  }
  const url = URL.createObjectURL(file);
  host.innerHTML = `<img src="${url}" alt="Original upload preview" />`;
}

function showMaskPreview(dataUrl) {
  const host = document.getElementById("maskPreview");
  if (!dataUrl) {
    host.innerHTML = '<span class="small">No mask preview available</span>';
    return;
  }
  host.innerHTML = `<img src="${dataUrl}" alt="Selected color mask preview" />`;
}

function setSelectedColors(colors) {
  selectedColors = new Set(colors);
  renderColorSwatches(latestAnalysis?.colors || []);
  updateSelectedColorSummary();
}

function updateSelectedColorSummary() {
  const host = document.getElementById("selectedColorSummary");
  const values = [...selectedColors];
  if (!values.length) {
    host.textContent = "No colors selected.";
    return;
  }
  host.textContent = `Selected for printing: ${values.join(", ")}`;
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
  const host = document.getElementById("colorSwatches");
  if (!colors.length) {
    host.innerHTML = '<span class="small">Analyze an image to populate selectable colors.</span>';
    return;
  }
  host.innerHTML = colors.map((color) => {
    const active = selectedColors.has(color.hex);
    const coverage = `${(color.coverage * 100).toFixed(1)}%`;
    return `
      <button
        type="button"
        class="swatch ${active ? "active" : ""}"
        data-color="${color.hex}"
        onclick="toggleColorSelection('${color.hex}')"
      >
        <span class="swatch-chip" style="background:${color.hex}"></span>
        <span class="swatch-text">${color.hex}</span>
        <span class="swatch-meta">${coverage} · ${color.pixel_count}px</span>
      </button>
    `;
  }).join("");
}

function toggleColorSelection(hex) {
  if (selectedColors.has(hex)) selectedColors.delete(hex);
  else selectedColors.add(hex);
  renderColorSwatches(latestAnalysis?.colors || []);
  updateSelectedColorSummary();
}

async function analyzeRasterImage() {
  const file = currentImageFile();
  if (!file) {
    appendLog("ERROR: Choose a PNG or JPG file first.");
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
      return;
    }

    latestAnalysis = json.analysis;
    renderColorSwatches(latestAnalysis.colors || []);
    if (!selectedColors.size) {
      setSelectedColors(guessInitialColors(latestAnalysis.colors || []));
    } else {
      renderColorSwatches(latestAnalysis.colors || []);
      updateSelectedColorSummary();
    }
    appendLog(`Detected ${latestAnalysis.colors.length} major colors in ${latestAnalysis.width}x${latestAnalysis.height} image.`);
  } catch (err) {
    appendLog(`ANALYZE FETCH ERROR: ${err}`);
  }
}

function buildRasterForm(file) {
  const form = new FormData();
  form.append("image", file);
  form.append("selected_colors", JSON.stringify([...selectedColors]));
  form.append("draw_feed", getNum("drawFeed"));
  form.append("travel_feed", getNum("travelFeed"));
  form.append("sample_step_deg", getNum("sampleStepDeg"));
  form.append("margin_percent", getNum("marginPercent"));
  form.append("fit_mode", document.getElementById("fitMode").value);
  form.append("invert_y", getBool("invertY") ? "1" : "0");
  form.append("include_comments", getBool("includeComments") ? "1" : "0");
  form.append("debug_pipeline", getBool("debugPipeline") ? "1" : "0");
  form.append("placement_scale", getNum("placementScale"));
  form.append("placement_offset_x", getNum("placementOffsetX"));
  form.append("placement_offset_y", getNum("placementOffsetY"));
  form.append("rotation_deg", getNum("rotationDeg"));
  form.append("line_thickness_mm", getNum("lineThicknessMm"));
  form.append("wall_count", getInt("wallCount"));
  form.append("infill_pattern", document.getElementById("infillPattern").value);
  form.append("infill_density", getNum("infillDensity"));
  if (getValue("infillSpacingMm") !== "") form.append("infill_spacing_mm", getNum("infillSpacingMm"));
  form.append("infill_angle_deg", getNum("infillAngleDeg"));
  form.append("outline_after_fill", getBool("outlineAfterFill") ? "1" : "0");
  if (getValue("minFillAreaMm2") !== "") form.append("min_fill_area_mm2", getNum("minFillAreaMm2"));
  if (getValue("minFillWidthMm") !== "") form.append("min_fill_width_mm", getNum("minFillWidthMm"));
  form.append("simplify_tolerance_mm", getNum("simplifyToleranceMm"));
  form.append("remove_duplicate_paths", getBool("removeDuplicatePaths") ? "1" : "0");
  form.append("small_shape_mode", document.getElementById("smallShapeMode").value);
  form.append("min_segment_length_mm", getNum("minSegmentLengthMm"));
  form.append("travel_optimization", document.getElementById("travelOptimization").value);
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

async function generateRasterGcode() {
  const file = currentImageFile();
  if (!file) {
    appendLog("ERROR: Choose a PNG or JPG file first.");
    return;
  }
  if (!selectedColors.size) {
    appendLog("ERROR: Select at least one detected color first.");
    return;
  }

  showOriginalPreview(file);
  try {
    const res = await fetch("/generate-image-gcode", {
      method: "POST",
      body: buildRasterForm(file)
    });
    const json = await res.json();
    if (!json.ok) {
      appendLog(`GENERATE ERROR: ${json.error}`);
      return;
    }

    latestGcode = json.gcode || [];
    latestPreview = json.preview || [];
    latestMask = json.mask || null;
    latestRegions = json.regions || null;

    document.getElementById("gcodeBox").value = latestGcode.join("\n");
    showMaskPreview(latestMask?.mask_preview_url || latestRegions?.boundary_preview_url || null);
    drawFlatPreview(latestPreview);
    drawBallPreview(latestPreview);

    appendLog(`Generated ${latestGcode.length} G-code lines from ${json.toolpath_count} toolpaths / ${json.point_count} plotted points.`);
    appendLog(`Selected colors: ${json.selected_colors.join(", ")}. Regions: ${latestRegions?.region_count || 0}, holes: ${latestRegions?.hole_count || 0}.`);
    appendLog(`Mask printable pixels: ${latestMask?.printable_pixel_count || 0}. Area extracted from mask: ${(latestRegions?.printable_area_px || 0).toFixed(1)} px².`);
    if (json.debug?.toolpath_counts) {
      const counts = json.debug.toolpath_counts;
      appendLog(`Generated toolpaths: walls ${counts.generated_fill_walls || 0}, infill ${counts.generated_infill_paths || 0}, outlines ${counts.generated_outline_paths || 0}.`);
    }
    await refreshState();
  } catch (err) {
    appendLog(`GENERATE FETCH ERROR: ${err}`);
  }
}

function setupCanvas(canvasId) {
  const canvas = document.getElementById(canvasId);
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(300, Math.floor(rect.width * window.devicePixelRatio));
  canvas.height = Math.max(300, Math.floor(rect.height * window.devicePixelRatio));
  return canvas.getContext("2d");
}

function previewStyle(kind, depth = 1) {
  const alpha = Math.max(0.18, Math.min(1, depth));
  if (kind === "outline") return { stroke: `rgba(59, 130, 246, ${alpha})`, width: 2.1, dash: [] };
  if (kind === "fill-wall") return { stroke: `rgba(245, 158, 11, ${alpha})`, width: 2.4, dash: [] };
  if (kind === "fill-infill") return { stroke: `rgba(45, 212, 191, ${alpha})`, width: 1.6, dash: [] };
  return { stroke: `rgba(148, 163, 184, ${alpha * 0.9})`, width: 1.1, dash: [6, 6] };
}

function drawFlatPreview(paths) {
  const ctx = setupCanvas("flatPreview");
  const w = ctx.canvas.width;
  const h = ctx.canvas.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#020617";
  ctx.fillRect(0, 0, w, h);

  const pad = 24 * window.devicePixelRatio;
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

  ctx.fillStyle = "rgba(37, 99, 235, 0.45)";
  ctx.fillRect(sx(0) - 1 * window.devicePixelRatio, top, 2 * window.devicePixelRatio, drawH);
  ctx.fillRect(left, sy(0) - 1 * window.devicePixelRatio, drawW, 2 * window.devicePixelRatio);

  ctx.fillStyle = "#94a3b8";
  ctx.font = `${11 * window.devicePixelRatio}px Arial`;
  ctx.fillText("X -180°", left, h - 7 * window.devicePixelRatio);
  ctx.fillText("X 0°", left + (180 * scale) - 18 * window.devicePixelRatio, h - 7 * window.devicePixelRatio);
  ctx.fillText("X +180°", w - left - 52 * window.devicePixelRatio, h - 7 * window.devicePixelRatio);
  ctx.fillText("Y +45°", 8 * window.devicePixelRatio, top + 4 * window.devicePixelRatio);
  ctx.fillText("Y 0°", 8 * window.devicePixelRatio, top + (45 * scale) + 4 * window.devicePixelRatio);
  ctx.fillText("Y -45°", 8 * window.devicePixelRatio, h - top);

  for (const entry of paths) {
    const path = entry.points || [];
    if (!path.length) continue;
    const style = previewStyle(entry.kind || "outline");
    ctx.beginPath();
    ctx.moveTo(sx(path[0].x), sy(path[0].y));
    for (let i = 1; i < path.length; i++) {
      ctx.lineTo(sx(path[i].x), sy(path[i].y));
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
    z
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
  sphere.addColorStop(0, "#e2e8f0");
  sphere.addColorStop(0.22, "#94a3b8");
  sphere.addColorStop(0.65, "#334155");
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
      points: path.map((point) => projectBallPoint(point, cx, cy, radius))
    });
  }
  projectedPaths.sort((a, b) => ((a.points[0]?.z || 0) - (b.points[0]?.z || 0)));

  for (const entry of projectedPaths) {
    const projected = entry.points;
    ctx.beginPath();
    for (let i = 0; i < projected.length; i++) {
      const p = projected[i];
      if (i === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    }
    const depth = projected.reduce((sum, p) => sum + p.z, 0) / projected.length;
    const style = previewStyle(entry.kind, 0.35 + ((depth + 1) * 0.325));
    ctx.setLineDash((style.dash || []).map((value) => value * window.devicePixelRatio));
    ctx.lineWidth = style.width * window.devicePixelRatio;
    ctx.strokeStyle = style.stroke;
    ctx.stroke();
  }
  ctx.setLineDash([]);
}

function downloadGcode() {
  const text = document.getElementById("gcodeBox").value;
  if (!text.trim()) {
    appendLog("ERROR: No G-code generated.");
    return;
  }
  const blob = new Blob([text], { type: "text/plain" });
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = "golfball_plotter_output.gcode";
  link.click();
  URL.revokeObjectURL(link.href);
}

document.getElementById("imageFile").addEventListener("change", () => {
  const file = currentImageFile();
  showOriginalPreview(file);
  if (!file) {
    latestAnalysis = null;
    latestMask = null;
    latestRegions = null;
    setSelectedColors([]);
    showMaskPreview(null);
    return;
  }
  setSelectedColors([]);
});

window.addEventListener("resize", () => {
  drawFlatPreview(latestPreview);
  drawBallPreview(latestPreview);
});

setupDerivedPenFieldSync();
syncDerivedPenFields(true);
setInterval(refreshState, 750);
refreshState();
showOriginalPreview(null);
showMaskPreview(null);
drawFlatPreview([]);
drawBallPreview([]);
appendLog("Controller loaded. Connect, calibrate, analyze a PNG/JPG, select colors, generate G-code, then run.");
