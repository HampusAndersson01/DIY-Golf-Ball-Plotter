let latestGcode = [];
  let latestPreview = [];
  let latestPrintModel = null;
  let latestViewboxBounds = null;

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
    return parseInt(document.getElementById(id).value || "0");
  }

  function getBool(id) {
    return document.getElementById(id).checked;
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

  function resetServoUiDefaults() {
    document.getElementById('penUpS').value = SERVER_DEFAULTS.penUpS;
    document.getElementById('penDownS').value = SERVER_DEFAULTS.penDownS;
    document.getElementById('penUpDwellMs').value = SERVER_DEFAULTS.penUpDwellMs;
    document.getElementById('penDownDwellMs').value = SERVER_DEFAULTS.penDownDwellMs;
    document.getElementById('servoRampEnabled').checked = SERVER_DEFAULTS.servoRampEnabled;
    document.getElementById('servoRampStep').value = SERVER_DEFAULTS.servoRampStep;
    document.getElementById('servoRampDelayMs').value = SERVER_DEFAULTS.servoRampDelayMs;
    document.getElementById('rawCommand').value = `M3 S${SERVER_DEFAULTS.penUpS}`;
    appendLog(`Reset servo UI fields to server defaults S${SERVER_DEFAULTS.penUpS}/S${SERVER_DEFAULTS.penDownS}.`);
  }

  function appendServoSettings(target) {
    target.servo_ramp_enabled = getBool('servoRampEnabled');
    target.servo_ramp_step = getInt('servoRampStep');
    target.servo_ramp_delay_ms = getNum('servoRampDelayMs');
    target.pen_up_dwell_ms = getNum('penUpDwellMs');
    target.pen_down_dwell_ms = getNum('penDownDwellMs');
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
      const res = await fetch('/state');
      const s = await res.json();
      document.getElementById('serialStatus').textContent = s.connected ? 'Connected' : 'Disconnected';
      document.getElementById('serialStatus').className = s.connected ? 'value ok-text' : 'value danger-text';
      document.getElementById('calStatus').textContent = s.calibrated ? 'Ready' : 'Not calibrated';
      document.getElementById('calStatus').className = s.calibrated ? 'value ok-text' : 'value warning';
      document.getElementById('runStatus').textContent = s.status || '-';
      document.getElementById('progressStatus').textContent = `${s.progress_done || 0} / ${s.progress_total || 0}`;
      if (s.defaults) {
        document.getElementById('servoDefaultsInfo').textContent =
          `Server defaults: pen up S${s.defaults.pen_up_s}, pen down S${s.defaults.pen_down_s}, PID ${s.server_pid}`;
      }

      const pct = s.progress_total ? Math.round((s.progress_done / s.progress_total) * 100) : 0;
      document.getElementById('progressFill').style.width = `${pct}%`;
    } catch (_) {}
  }

  function sendCommand(command) { return api('/command', { command }); }
  function sendRaw() { return sendCommand(document.getElementById('rawCommand').value); }
  function softReset() { return api('/reset'); }

  async function runSvgPipelineSelfTest() {
    try {
      const res = await fetch('/self-test-svg-pipeline', { method: 'POST' });
      const json = await res.json();
      if (!json.ok) {
        appendLog(`SVG SELF-TEST ERROR: ${json.error}`);
        return;
      }
      appendLog(`SVG self-test passed: ${json.summary.passed} checks.`);
      for (const line of (json.summary.messages || [])) {
        appendLog(`SELF-TEST: ${line}`);
      }
    } catch (err) {
      appendLog(`SVG SELF-TEST FETCH ERROR: ${err}`);
    }
  }

  function applyMachineConfig() {
    return api('/apply-config', {
      x_max_feed: getNum('xMaxFeed'),
      y_max_feed: getNum('yMaxFeed'),
      x_acceleration: getNum('xAcceleration'),
      y_acceleration: getNum('yAcceleration')
    });
  }

  function penUp() {
    return api('/pen-up', appendServoSettings({
      s: getInt('penUpS'),
    }));
  }

  function penDown() {
    return api('/pen-down', appendServoSettings({
      s: getInt('penDownS'),
    }));
  }

  function penTest() {
    return api('/pen-test', appendServoSettings({
      up_s: getInt('penUpS'),
      down_s: getInt('penDownS'),
    }));
  }

  function servoOff() { return api('/servo-off'); }
  function jogX(degrees) { return api('/jog', { axis: 'X', degrees, feed: getNum('travelFeed') }); }
  function jogY(degrees) { return api('/jog', { axis: 'Y', degrees, feed: getNum('travelFeed') }); }
  function zeroPosition() { return api('/zero-position'); }
  function goHome() {
    return api('/go-home', appendServoSettings({
      pen_up_s: getInt('penUpS'),
      travel_feed: getNum('travelFeed')
    }));
  }
  function markCalibrated() { return api('/mark-calibrated'); }
  function clearCalibrated() { return api('/clear-calibrated'); }
  function runGcode() { return api('/run-gcode'); }
  function pauseRun() { return api('/pause'); }
  function resumeRun() { return api('/resume'); }
  function stopRun() { return api('/stop'); }

  async function uploadAndGenerate() {
    const file = document.getElementById('svgFile').files[0];
    if (!file) {
      appendLog('ERROR: Choose an SVG file first.');
      return;
    }

    const text = await file.text();
    document.getElementById('svgPreview').innerHTML = text;

    const form = new FormData();
    form.append('svg', file);
    form.append('draw_feed', getNum('drawFeed'));
    form.append('travel_feed', getNum('travelFeed'));
    form.append('sample_step_deg', getNum('sampleStepDeg'));
    form.append('margin_percent', getNum('marginPercent'));
    form.append('fit_mode', document.getElementById('fitMode').value);
    form.append('parser_mode', document.getElementById('parserMode').value);
    form.append('color_mapping_mode', getBool('colorMappingMode') ? '1' : '0');
    form.append('invert_y', document.getElementById('invertY').checked ? '1' : '0');
    form.append('include_comments', document.getElementById('includeComments').checked ? '1' : '0');
    form.append('debug_pipeline', getBool('debugPipeline') ? '1' : '0');
    form.append('placement_scale', getNum('placementScale'));
    form.append('placement_offset_x', getNum('placementOffsetX'));
    form.append('placement_offset_y', getNum('placementOffsetY'));
    form.append('rotation_deg', getNum('rotationDeg'));
    form.append('line_thickness_mm', getNum('lineThicknessMm'));
    form.append('enable_fill', getBool('enableFill') ? '1' : '0');
    form.append('fill_only_dark_svg_fills', getBool('fillOnlyDarkSvgFills') ? '1' : '0');
    form.append('trace_stroke_only_paths', getBool('traceStrokeOnlyPaths') ? '1' : '0');
    form.append('fill_mode', document.getElementById('fillMode').value);
    form.append('wall_count', getInt('wallCount'));
    form.append('infill_pattern', document.getElementById('infillPattern').value);
    form.append('infill_density', getNum('infillDensity'));
    form.append('infill_spacing_mm', getNum('infillSpacingMm'));
    form.append('infill_angle_deg', getNum('infillAngleDeg'));
    form.append('outline_after_fill', getBool('outlineAfterFill') ? '1' : '0');
    form.append('min_fill_area_mm2', getNum('minFillAreaMm2'));
    form.append('min_fill_width_mm', getNum('minFillWidthMm'));
    form.append('simplify_tolerance_mm', getNum('simplifyToleranceMm'));
    form.append('remove_duplicate_paths', getBool('removeDuplicatePaths') ? '1' : '0');
    form.append('small_shape_mode', document.getElementById('smallShapeMode').value);
    form.append('min_segment_length_mm', getNum('minSegmentLengthMm'));
    form.append('travel_optimization', document.getElementById('travelOptimization').value);
    form.append('pen_up_s', getInt('penUpS'));
    form.append('pen_down_s', getInt('penDownS'));
    form.append('servo_ramp_enabled', getBool('servoRampEnabled') ? '1' : '0');
    form.append('servo_ramp_step', getInt('servoRampStep'));
    form.append('servo_ramp_delay_ms', getNum('servoRampDelayMs'));
    form.append('pen_up_dwell_ms', getNum('penUpDwellMs'));
    form.append('pen_down_dwell_ms', getNum('penDownDwellMs'));

    try {
      const res = await fetch('/generate-gcode', { method: 'POST', body: form });
      const json = await res.json();
      if (!json.ok) {
        appendLog(`ERROR: ${json.error}`);
        return;
      }

      latestGcode = json.gcode;
      latestPreview = json.preview || [];
      latestPrintModel = json.print_model || null;
      latestViewboxBounds = json.viewbox_bounds || null;
      document.getElementById('gcodeBox').value = latestGcode.join('\n');
      renderClassifiedPreview(latestPrintModel, latestViewboxBounds);
      drawFlatPreview(latestPreview);
      drawBallPreview(latestPreview);
      appendLog(`Generated ${latestGcode.length} G-code lines from ${json.toolpath_count} toolpaths / ${json.point_count} plotted points.`);
      const classificationCounts = json.print_model?.metadata?.classificationCounts || {};
      appendLog(`SVG classification: dark fills ${classificationCounts.dark_filled_polygons || 0}, light cutouts ${classificationCounts.light_cutout_polygons || 0}, transparent cutouts ${classificationCounts.transparent_cutout_polygons || 0}, stroke-only ${classificationCounts.stroke_only_paths || 0}, ignored ${classificationCounts.ignored_paths || 0}.`);
      if (json.debug?.toolpath_counts) {
        const counts = json.debug.toolpath_counts;
        appendLog(`Generated toolpaths: walls ${counts.generated_fill_walls || 0}, infill ${counts.generated_infill_paths || 0}, outlines ${counts.generated_outline_paths || 0}.`);
      }
      for (const warning of (json.print_model?.warnings || [])) {
        appendLog(`WARNING: ${warning}`);
      }
      for (const diagnostic of (json.print_model?.diagnostics || [])) {
        appendLog(`DIAGNOSTIC: ${diagnostic}`);
      }
      await refreshState();
    } catch (err) {
      appendLog(`GENERATE ERROR: ${err}`);
    }
  }

  function renderClassifiedPreview(printModel, viewboxBounds) {
    const host = document.getElementById('classifiedPreview');
    if (!host) return;
    if (!printModel) {
      host.innerHTML = '<span class="small">No classified SVG regions available</span>';
      return;
    }

    const fills = printModel.fills || [];
    const cutouts = printModel.cutouts || [];
    const strokes = printModel.strokes || [];
    const computed = printModel.computed_bounds || {};
    const vb = viewboxBounds || {};
    const minX = Number.isFinite(vb.min_x) ? vb.min_x : (Number.isFinite(computed.min_x) ? computed.min_x : 0);
    const minY = Number.isFinite(vb.min_y) ? vb.min_y : (Number.isFinite(computed.min_y) ? computed.min_y : 0);
    const maxX = Number.isFinite(vb.max_x) ? vb.max_x : (Number.isFinite(computed.max_x) ? computed.max_x : minX + 100);
    const maxY = Number.isFinite(vb.max_y) ? vb.max_y : (Number.isFinite(computed.max_y) ? computed.max_y : minY + 100);
    const width = Math.max(1, maxX - minX);
    const height = Math.max(1, maxY - minY);

    const fillMarkup = fills.map((region) => {
      const mergedPath = [...(region.paths || []), ...(region.holes || [])].join(' ');
      return `<path d="${escapeHtml(mergedPath)}" fill="rgba(245, 158, 11, 0.7)" stroke="#f59e0b" stroke-width="${Math.max(width, height) * 0.0015}" fill-rule="evenodd"/>`;
    }).join('');

    const cutoutMarkup = cutouts.map((region) => {
      const mergedPath = [...(region.paths || []), ...(region.holes || [])].join(' ');
      return `<path d="${escapeHtml(mergedPath)}" fill="rgba(239, 68, 68, 0.18)" stroke="#ef4444" stroke-width="${Math.max(width, height) * 0.0015}" fill-rule="evenodd"/>`;
    }).join('');

    const strokeMarkup = strokes.map((region) =>
      `<path d="${escapeHtml(region.path || '')}" fill="none" stroke="rgba(59, 130, 246, 0.95)" stroke-width="${Math.max(region.strokeWidth || 1, Math.max(width, height) * 0.0012)}" stroke-linecap="round" stroke-linejoin="round"/>`
    ).join('');

    host.innerHTML = `
      <svg viewBox="${minX} ${minY} ${width} ${height}" xmlns="http://www.w3.org/2000/svg" aria-label="Classified SVG regions">
        <rect x="${minX}" y="${minY}" width="${width}" height="${height}" fill="#020617"/>
        <g>${fillMarkup}</g>
        <g>${cutoutMarkup}</g>
        <g>${strokeMarkup}</g>
      </svg>
    `;
  }

  function escapeHtml(text) {
    return String(text || '')
      .replaceAll('&', '&amp;')
      .replaceAll('<', '&lt;')
      .replaceAll('>', '&gt;')
      .replaceAll('\"', '&quot;');
  }

  function setupCanvas(canvasId) {
    const canvas = document.getElementById(canvasId);
    const rect = canvas.getBoundingClientRect();
    canvas.width = Math.max(300, Math.floor(rect.width * window.devicePixelRatio));
    canvas.height = Math.max(300, Math.floor(rect.height * window.devicePixelRatio));
    return canvas.getContext('2d');
  }

  function previewStyle(kind, depth = 1) {
    const alpha = Math.max(0.18, Math.min(1, depth));
    if (kind === 'outline') return { stroke: `rgba(59, 130, 246, ${alpha})`, width: 2.1, dash: [] };
    if (kind === 'fill-wall') return { stroke: `rgba(245, 158, 11, ${alpha})`, width: 2.4, dash: [] };
    if (kind === 'fill-infill') return { stroke: `rgba(45, 212, 191, ${alpha})`, width: 1.6, dash: [] };
    return { stroke: `rgba(148, 163, 184, ${alpha * 0.9})`, width: 1.1, dash: [6, 6] };
  }

  function drawFlatPreview(paths) {
    const ctx = setupCanvas('flatPreview');
    const w = ctx.canvas.width;
    const h = ctx.canvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#020617';
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

    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1 * window.devicePixelRatio;
    ctx.strokeRect(left, top, drawW, drawH);

    ctx.fillStyle = 'rgba(37, 99, 235, 0.45)';
    ctx.fillRect(sx(0) - 1 * window.devicePixelRatio, top, 2 * window.devicePixelRatio, drawH);
    ctx.fillRect(left, sy(0) - 1 * window.devicePixelRatio, drawW, 2 * window.devicePixelRatio);

    ctx.fillStyle = '#94a3b8';
    ctx.font = `${11 * window.devicePixelRatio}px Arial`;
    ctx.fillText('X -180°', left, h - 7 * window.devicePixelRatio);
    ctx.fillText('X 0°', cxLabel(w, scale, left), h - 7 * window.devicePixelRatio);
    ctx.fillText('X +180°', w - left - 48 * window.devicePixelRatio, h - 7 * window.devicePixelRatio);
    ctx.fillText('Y +45°', 8 * window.devicePixelRatio, top + 4 * window.devicePixelRatio);
    ctx.fillText('Y 0°', 8 * window.devicePixelRatio, cyLabel(h, scale, top));
    ctx.fillText('Y -45°', 8 * window.devicePixelRatio, h - top);

    for (const entry of paths) {
      const path = entry.points || [];
      if (!path.length) continue;
      const style = previewStyle(entry.kind || 'outline');
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

  function cxLabel(width, scale, left) {
    return left + (180 * scale) - 18 * window.devicePixelRatio;
  }

  function cyLabel(height, scale, top) {
    return top + (45 * scale) + 4 * window.devicePixelRatio;
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
    const ctx = setupCanvas('ballPreview');
    const w = ctx.canvas.width;
    const h = ctx.canvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#020617';
    ctx.fillRect(0, 0, w, h);

    const radius = Math.min(w, h) * 0.38;
    const cx = w / 2;
    const cy = h / 2;

    const sphere = ctx.createRadialGradient(cx - radius * 0.35, cy - radius * 0.35, radius * 0.1, cx, cy, radius);
    sphere.addColorStop(0, '#e2e8f0');
    sphere.addColorStop(0.22, '#94a3b8');
    sphere.addColorStop(0.65, '#334155');
    sphere.addColorStop(1, '#0f172a');
    ctx.fillStyle = sphere;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = '#475569';
    ctx.lineWidth = 1 * window.devicePixelRatio;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.stroke();

    ctx.strokeStyle = 'rgba(148, 163, 184, 0.35)';
    ctx.lineWidth = 1.2 * window.devicePixelRatio;
    ctx.beginPath();
    ctx.arc(cx, cy, radius * 0.56, 0, Math.PI * 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(cx - radius, cy);
    ctx.lineTo(cx + radius, cy);
    ctx.moveTo(cx, cy - radius);
    ctx.lineTo(cx, cy + radius);
    ctx.stroke();

    ctx.strokeStyle = 'rgba(148, 163, 184, 0.18)';
    ctx.lineWidth = 1 * window.devicePixelRatio;
    for (const lonDeg of [-120, -60, 0, 60, 120]) {
      ctx.beginPath();
      for (let latDeg = -45; latDeg <= 45; latDeg += 3) {
        const p = projectBallPoint({ x: lonDeg, y: latDeg }, cx, cy, radius);
        if (latDeg === -45) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
      }
      ctx.stroke();
    }
    ctx.beginPath();
    for (let lonDeg = -180; lonDeg <= 180; lonDeg += 4) {
      const p = projectBallPoint({ x: lonDeg, y: 0 }, cx, cy, radius);
      if (lonDeg === -180) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();

    const projectedPaths = [];
    for (const entry of paths) {
      const path = entry.points || [];
      if (!path.length) continue;
      projectedPaths.push({
        kind: entry.kind || 'outline',
        points: path.map((point) => projectBallPoint(point, cx, cy, radius))
      });
    }
    projectedPaths.sort((a, b) => ((a.points[0]?.z || 0) - (b.points[0]?.z || 0)));

    for (const entry of projectedPaths) {
      const projected = entry.points;
      ctx.beginPath();
      for (let i = 0; i < projected.length; i++) {
        const p = projected[i];
        if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
      }
      const depth = projected.reduce((sum, p) => sum + p.z, 0) / projected.length;
      const style = previewStyle(entry.kind, 0.35 + ((depth + 1) * 0.325));
      ctx.setLineDash((style.dash || []).map((value) => value * window.devicePixelRatio));
      ctx.lineWidth = style.width * window.devicePixelRatio;
      ctx.strokeStyle = style.stroke;
      ctx.stroke();
    }
    ctx.setLineDash([]);

    ctx.fillStyle = 'rgba(255, 255, 255, 0.38)';
    ctx.beginPath();
    ctx.arc(cx - radius * 0.32, cy - radius * 0.34, radius * 0.16, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = '#cbd5e1';
    ctx.font = `${11 * window.devicePixelRatio}px Arial`;
    ctx.fillText('Printer view rotation', cx - 38 * window.devicePixelRatio, cy + radius + 16 * window.devicePixelRatio);
    ctx.fillText('Right side = bottom', cx - 42 * window.devicePixelRatio, cy + radius + 30 * window.devicePixelRatio);
    ctx.fillText('X0° / Y0°', cx - 24 * window.devicePixelRatio, cy + 4 * window.devicePixelRatio);
  }

  function downloadGcode() {
    const text = document.getElementById('gcodeBox').value;
    if (!text.trim()) {
      appendLog('ERROR: No G-code generated.');
      return;
    }
    const blob = new Blob([text], { type: 'text/plain' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'golfball_plotter_output.gcode';
    a.click();
    URL.revokeObjectURL(a.href);
  }

  window.addEventListener('resize', () => {
    drawFlatPreview(latestPreview);
    drawBallPreview(latestPreview);
  });
  setInterval(refreshState, 750);
  refreshState();
  appendLog('Controller loaded. Connect, apply settings, upload SVG, calibrate, then run.');
