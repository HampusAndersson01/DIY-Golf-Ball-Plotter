import { startTransition, useEffect, useEffectEvent, useMemo, useRef, useState } from 'react'
import type { ChangeEvent } from 'react'

import { analyzeImage, apiConfig, fetchBootstrap, fetchState, generateDiagnosticGcode, generateImageGcode, postJson } from './api/client'
import { CalibrationPatternPanel } from './components/calibration/CalibrationPatternPanel'
import { XAxisCalibrationPanel } from './components/calibration/XAxisCalibrationPanel'
import { AppShell } from './components/layout/AppShell'
import { StepNav } from './components/layout/StepNav'
import { TopStatusBar } from './components/layout/TopStatusBar'
import { ColorPickerPanel } from './components/image/ColorPickerPanel'
import { ImageImportCard } from './components/image/ImageImportCard'
import { PenSettingsCard } from './components/image/PenSettingsCard'
import { AdvancedDrawer } from './components/job/AdvancedDrawer'
import { GcodePanel } from './components/job/GcodePanel'
import { JobSummaryPanel } from './components/job/JobSummaryPanel'
import { LogsPanel } from './components/job/LogsPanel'
import { CalibrationCard } from './components/machine/CalibrationCard'
import { MachineCard } from './components/machine/MachineCard'
import { ManualControlCard } from './components/machine/ManualControlCard'
import { RunControls } from './components/machine/RunControls'
import { PreviewWorkspace } from './components/preview/PreviewWorkspace'
import { getProgressPercent } from './components/preview/previewMath'
import type { JobSummary, MachineState, PreviewPath } from './api/types'
import type { SettingsState } from './store/appStore'
import { useAppStore } from './store/appStore'

function App() {
  const busy = useAppStore((state) => state.busy)
  const config = useAppStore((state) => state.config)
  const initialize = useAppStore((state) => state.initialize)
  const pushToast = useAppStore((state) => state.pushToast)
  const appendLog = useAppStore((state) => state.appendLog)

  const [bootstrapError, setBootstrapError] = useState<string | null>(null)

  useEffect(() => {
    let active = true
    const load = async () => {
      try {
        const nextConfig = await fetchBootstrap()
        if (!active) return
        initialize(nextConfig)
      } catch (error) {
        if (!active) return
        const message = String(error)
        setBootstrapError(message)
        appendLog(`Bootstrap failed: ${message}`)
        pushToast(message, 'error')
      }
    }
    void load()
    return () => {
      active = false
    }
  }, [appendLog, initialize, pushToast])

  if (busy.bootstrapping) {
    return <BootMessage title="Loading dashboard" detail="Fetching backend defaults and API bootstrap from Flask." />
  }

  if (bootstrapError || !config) {
    return <BootMessage title="Dashboard bootstrap failed" detail={bootstrapError ?? 'Missing frontend config.'} />
  }

  return <DashboardApp />
}

function DashboardApp() {
  const machine = useAppStore((state) => state.machine)
  const settings = useAppStore((state) => state.settings)
  const imageFile = useAppStore((state) => state.imageFile)
  const imagePreviewUrl = useAppStore((state) => state.imagePreviewUrl)
  const analysis = useAppStore((state) => state.analysis)
  const selectedColors = useAppStore((state) => state.selectedColors)
  const preview = useAppStore((state) => state.preview)
  const maskPreviewUrl = useAppStore((state) => state.maskPreviewUrl)
  const gcode = useAppStore((state) => state.gcode)
  const summary = useAppStore((state) => state.summary)
  const logs = useAppStore((state) => state.logs)
  const toasts = useAppStore((state) => state.toasts)
  const previewMode = useAppStore((state) => state.previewMode)
  const progressFilter = useAppStore((state) => state.progressFilter)
  const showTravel = useAppStore((state) => state.showTravel)
  const showCompare = useAppStore((state) => state.showCompare)
  const drawerTab = useAppStore((state) => state.drawerTab)
  const advancedOpen = useAppStore((state) => state.advancedOpen)
  const viewPreset = useAppStore((state) => state.viewPreset)
  const busy = useAppStore((state) => state.busy)
  const setMachine = useAppStore((state) => state.setMachine)
  const setImageFile = useAppStore((state) => state.setImageFile)
  const setAnalysis = useAppStore((state) => state.setAnalysis)
  const toggleColor = useAppStore((state) => state.toggleColor)
  const setPreviewPayload = useAppStore((state) => state.setPreviewPayload)
  const setPreviewMode = useAppStore((state) => state.setPreviewMode)
  const setProgressFilter = useAppStore((state) => state.setProgressFilter)
  const setShowTravel = useAppStore((state) => state.setShowTravel)
  const setShowCompare = useAppStore((state) => state.setShowCompare)
  const setDrawerTab = useAppStore((state) => state.setDrawerTab)
  const setViewPreset = useAppStore((state) => state.setViewPreset)
  const setBusy = useAppStore((state) => state.setBusy)
  const appendLog = useAppStore((state) => state.appendLog)
  const pushToast = useAppStore((state) => state.pushToast)
  const dismissToast = useAppStore((state) => state.dismissToast)
  const [generationDurationMs, setGenerationDurationMs] = useState<number | null>(null)
  const restoredPersistedPreviewRef = useRef(false)

  const readySettings = settings
  const progressPercent = getProgressPercent(machine)
  const runReady = Boolean(machine?.connected && machine?.calibrated && gcode.length && !machine?.y_loop_test?.enabled && !busy.running)
  const currentSettings = useMemo(() => readySettings, [readySettings])
  const currentPath = useMemo(
    () => preview.find((path) => path.id === machine?.current_path_id) ?? null,
    [machine?.current_path_id, preview],
  )
  const currentKind = currentPath ? formatKind(currentPath.kind) : 'Idle'
  const elapsedSeconds = getElapsedSeconds(machine)
  const remainingSeconds = getRemainingSeconds(machine, summary, progressPercent, elapsedSeconds)

  const onKeyboardPause = useEffectEvent(async () => {
    if (!currentSettings) return
    if (machine?.paused) await handleResume()
    else await handlePause()
  })

  useEffect(() => {
    let active = true
    const load = async () => {
      try {
        const nextState = await fetchState()
        if (!active) return
        startTransition(() => {
          setMachine(nextState)
          hydrateGeneratedPreviewFromMachineOnce(nextState)
        })
      } catch (error) {
        if (!active) return
        appendLog(`State poll failed: ${String(error)}`)
      }
    }
    void load()
    const timer = window.setInterval(load, 750)
    return () => {
      active = false
      window.clearInterval(timer)
    }
  }, [appendLog, setMachine])

  useEffect(() => {
    const timers = toasts.map((toast) => window.setTimeout(() => dismissToast(toast.id), 2800))
    return () => timers.forEach((timer) => window.clearTimeout(timer))
  }, [dismissToast, toasts])

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'f' || event.key === 'F') {
        event.preventDefault()
        window.dispatchEvent(new Event('preview-fit'))
      }
      if (event.key === 'r' || event.key === 'R') {
        event.preventDefault()
        window.dispatchEvent(new Event('preview-reset'))
      }
      if (event.key === 'Escape') {
        setShowCompare(false)
      }
      if (event.code === 'Space' && machine?.running) {
        event.preventDefault()
        void onKeyboardPause()
      }
    }
    window.addEventListener('keydown', onKeyDown)
    return () => window.removeEventListener('keydown', onKeyDown)
  }, [machine?.running, setShowCompare])

  if (!readySettings) {
    return <BootMessage title="Loading dashboard" detail="Preparing local operator state." />
  }

  const settingsState = readySettings

  async function refreshMachine() {
    const nextState = await fetchState()
    setMachine(nextState)
  }

  function hydrateGeneratedPreviewFromMachineOnce(nextState: MachineState) {
    if (restoredPersistedPreviewRef.current) {
      return
    }

    const hydratedPreview = sanitizePreviewPaths(
      Array.isArray(nextState.last_preview) ? (nextState.last_preview as Array<Record<string, unknown>>) : [],
    )
    const hydratedGcode = Array.isArray(nextState.last_gcode)
      ? nextState.last_gcode.filter((line): line is string => typeof line === 'string')
      : []
    const hydratedSummary = nextState.last_summary ?? null

    if (!hydratedPreview.length && !hydratedGcode.length && !hydratedSummary) {
      return
    }

    restoredPersistedPreviewRef.current = true
    setPreviewPayload({
      preview: hydratedPreview,
      maskPreviewUrl: null,
      gcode: hydratedGcode,
      summary: hydratedSummary,
      calibrationPattern: null,
      xAxisCalibrationPattern: null,
    })
  }

  async function callEndpoint(endpoint: string, body: unknown, successMessage?: string) {
    const response = await postJson<Record<string, never>>(endpoint, body)
    if (response.command) appendLog(`> ${response.command}`)
    if (response.response) appendLog(`< ${response.response}`)
    if (successMessage) pushToast(successMessage, 'success')
    await refreshMachine()
  }

  async function handleConnect() {
    setBusy('connecting', true)
    try {
      await callEndpoint(apiConfig.endpoints.connect, {}, 'Controller connected.')
    } catch (error) {
      pushToast(String(error), 'error')
      appendLog(`Connect failed: ${String(error)}`)
    } finally {
      setBusy('connecting', false)
    }
  }

  async function handleApplyConfig() {
    try {
      await callEndpoint(
        apiConfig.endpoints.applyConfig,
        {
          x_max_feed: settingsState.xMaxFeed,
          y_max_feed: settingsState.yMaxFeed,
          x_acceleration: settingsState.xAcceleration,
          y_acceleration: settingsState.yAcceleration,
        },
        'Machine settings applied.',
      )
    } catch (error) {
      pushToast(String(error), 'error')
    }
  }

  async function handleAnalyze() {
    if (!imageFile) {
      pushToast('Select a PNG or JPG first.', 'error')
      return
    }
    setBusy('analyzing', true)
    try {
      const formData = new FormData()
      formData.append('image', imageFile)
      formData.append('max_colors', String(settingsState.maxColors))
      formData.append('simplify_colors', settingsState.simplifyColors ? '1' : '0')
      const result = await analyzeImage(formData)
      setAnalysis(result)
      pushToast('Colors analyzed.', 'success')
    } catch (error) {
      pushToast(String(error), 'error')
      appendLog(`Analyze failed: ${String(error)}`)
    } finally {
      setBusy('analyzing', false)
    }
  }

  async function handleGenerate() {
    if (!imageFile || !selectedColors.length) {
      pushToast('Load an image and select at least one printable color.', 'error')
      return
    }
    setBusy('generating', true)
    const startedAt = performance.now()
    try {
      const normalizedSettings = normalizeGenerateSettings(settingsState)
      const payload = await generateImageGcode(buildGenerateFormData(imageFile, normalizedSettings, selectedColors))
      const effectiveSettings = normalizeEffectiveSettings(payload.effective_settings, normalizedSettings)
      const preview = sanitizePreviewPaths(payload.preview)
      setPreviewPayload({
        preview,
        maskPreviewUrl: payload.mask_preview,
        gcode: payload.gcode,
        summary: payload.summary,
        calibrationPattern: payload.calibrationPattern ?? null,
        xAxisCalibrationPattern: payload.xAxisCalibrationPattern ?? null,
      })
      setGenerationDurationMs(performance.now() - startedAt)
      appendLog(`Generated ${payload.gcode.length} G-code lines across ${preview.length} preview paths.`)
      appendLog(
        `Effective slicer settings: pen ${effectiveSettings.line_thickness_mm.toFixed(2)} mm, infill spacing ${effectiveSettings.infill_spacing_mm.toFixed(2)} mm, custom spacing ${effectiveSettings.custom_infill_spacing ? 'on' : 'off'}.`,
      )
      pushToast('G-code generated.', 'success')
      await refreshMachine()
    } catch (error) {
      pushToast(String(error), 'error')
      appendLog(`Generate failed: ${String(error)}`)
    } finally {
      setBusy('generating', false)
    }
  }

  async function handlePenUp() {
    await callEndpoint(apiConfig.endpoints.penUp, servoPayload(settingsState, settingsState.penUpS), 'Pen up.')
  }

  async function handlePenDown() {
    await callEndpoint(apiConfig.endpoints.penDown, servoPayload(settingsState, settingsState.penDownS), 'Pen down.')
  }

  async function handleGenerateCalibrationPattern() {
    setBusy('generating', true)
    const formData = new FormData()
    formData.append('pattern', '3x3_squares')
    formData.append('mode', 'fill_then_cleanup')
    formData.append('line_thickness_mm', String(settingsState.lineThicknessMm))
    formData.append('draw_feed', String(settingsState.drawFeed))
    formData.append('travel_feed', String(settingsState.travelFeed))
    formData.append('wall_count', String(settingsState.wallCount))
    try {
      const payload = await generateDiagnosticGcode(formData)
      const preview = sanitizePreviewPaths(payload.preview)
      setPreviewPayload({
        preview,
        maskPreviewUrl: null,
        gcode: payload.gcode,
        summary: payload.summary,
        calibrationPattern: payload.calibrationPattern ?? null,
        xAxisCalibrationPattern: payload.xAxisCalibrationPattern ?? null,
      })
      appendLog(`Generated diagnostic pattern ${payload.calibrationPattern?.pattern ?? '3x3_squares'} with ${preview.length} preview paths.`)
      pushToast('Calibration test pattern generated.', 'success')
      await refreshMachine()
    } catch (error) {
      pushToast(String(error), 'error')
      appendLog(`Calibration pattern generation failed: ${String(error)}`)
    } finally {
      setBusy('generating', false)
    }
  }

  async function handleGenerateXAxisCalibrationPattern() {
    setBusy('generating', true)
    const formData = new FormData()
    formData.append('pattern', 'x_axis_rotation_ticks')
    formData.append('draw_feed', String(settingsState.drawFeed))
    formData.append('travel_feed', String(settingsState.travelFeed))
    formData.append('line_thickness_mm', String(settingsState.lineThicknessMm))
    try {
      const payload = await generateDiagnosticGcode(formData)
      const preview = sanitizePreviewPaths(payload.preview)
      setPreviewPayload({
        preview,
        maskPreviewUrl: null,
        gcode: payload.gcode,
        summary: payload.summary,
        calibrationPattern: payload.calibrationPattern ?? null,
        xAxisCalibrationPattern: payload.xAxisCalibrationPattern ?? null,
      })
      appendLog(`Generated X rotary calibration pattern with ${preview.length} preview paths.`)
      pushToast('X rotary calibration pattern generated.', 'success')
      await refreshMachine()
    } catch (error) {
      pushToast(String(error), 'error')
      appendLog(`X rotary calibration generation failed: ${String(error)}`)
    } finally {
      setBusy('generating', false)
    }
  }

  async function handleGoHome() {
    await callEndpoint(apiConfig.endpoints.goHome, {
      pen_up_s: settingsState.penUpS,
      travel_feed: settingsState.travelFeed,
      pen_up_dwell_ms: settingsState.penUpDwellMs,
      servo_ramp_enabled: settingsState.servoRampEnabled,
      servo_ramp_step: settingsState.servoRampStep,
      servo_ramp_delay_ms: settingsState.servoRampDelayMs,
    })
  }

  async function handleJog(axis: 'X' | 'Y', degrees: number) {
    await callEndpoint(apiConfig.endpoints.jog, { axis, degrees, feed: settingsState.travelFeed })
  }

  async function handleCalibrate() {
    if (!window.confirm('Confirm the pen is physically positioned at the center of the ball.')) return
    setBusy('calibrating', true)
    try {
      await callEndpoint(apiConfig.endpoints.zeroAndMarkCalibrated, {}, 'Origin locked.')
    } catch (error) {
      pushToast(String(error), 'error')
    } finally {
      setBusy('calibrating', false)
    }
  }

  async function handleClearCalibrated() {
    await callEndpoint(apiConfig.endpoints.clearCalibrated, {}, 'Calibration cleared.')
  }

  async function handleTestStepperHoldPolicy() {
    if (!machine?.connected) {
      pushToast('Connect the controller first.', 'error')
      return
    }
    try {
      if (machine.calibrated) {
        if (!window.confirm('Calibration is currently locked. Clear calibration first so the release test can actually release the motors?')) return
        await handleClearCalibrated()
      }
      await callEndpoint(apiConfig.endpoints.applyStepperHoldPolicy, {}, 'Release policy applied.')
      window.alert('Verify X and Y can be moved by hand.')
      if (!window.confirm('Position the pen at origin and apply calibration now?')) return
      await handleCalibrate()
      window.alert('Verify X and Y resist manual movement.')
      if (!window.confirm('Clear calibration and re-test release?')) return
      await handleClearCalibrated()
      window.alert('Verify X and Y can be moved by hand again.')
    } catch (error) {
      pushToast(String(error), 'error')
      appendLog(`Stepper hold policy test failed: ${String(error)}`)
    }
  }

  async function handleRun() {
    if (!window.confirm('Start the generated job on the connected plotter?')) return
    setBusy('running', true)
    try {
      await callEndpoint(apiConfig.endpoints.runGcode, {}, 'Job started.')
    } catch (error) {
      pushToast(String(error), 'error')
      appendLog(`Run failed: ${String(error)}`)
    } finally {
      setBusy('running', false)
    }
  }

  async function handlePause() {
    await callEndpoint(apiConfig.endpoints.pause, {}, 'Pause requested.')
  }

  async function handleResume() {
    await callEndpoint(apiConfig.endpoints.resume, {}, 'Resume requested.')
  }

  async function handleStop() {
    await callEndpoint(apiConfig.endpoints.stop, {}, 'Stop requested.')
  }

  async function handleToggleYLoop() {
    try {
      const enabled = Boolean(machine?.y_loop_test?.enabled)
      if (enabled) {
        await callEndpoint(apiConfig.endpoints.yLoopStop, {}, 'Y loop test stopped.')
        return
      }
      await callEndpoint(
        apiConfig.endpoints.yLoopStart,
        {
          distance: settingsState.yLoopDistance,
          feedrate: settingsState.yLoopFeedrate,
          dwell_sec: settingsState.yLoopDwellSec,
          pen_up_s: settingsState.penUpS,
          pen_up_dwell_ms: settingsState.penUpDwellMs,
        },
        'Y loop test started.',
      )
    } catch (error) {
      pushToast(String(error), 'error')
      appendLog(`Y loop test failed: ${String(error)}`)
    }
  }

  function handleFileChange(event: ChangeEvent<HTMLInputElement>) {
    const file = event.target.files?.[0] ?? null
    const previewUrl = file ? URL.createObjectURL(file) : null
    if (imagePreviewUrl) URL.revokeObjectURL(imagePreviewUrl)
    setImageFile(file, previewUrl)
  }

  function jumpTo(step: string) {
    document.querySelector(`[data-step-anchor="${step}"]`)?.scrollIntoView({ block: 'nearest' })
  }

  return (
    <>
      <AppShell
        leftRail={
          <div className="rail-stack">
            <div data-step-anchor="connect">
              <MachineCard onApplyConfig={handleApplyConfig} onConnect={handleConnect} />
            </div>
            <div data-step-anchor="calibrate">
              <CalibrationCard machine={machine} onCalibrate={handleCalibrate} onClear={handleClearCalibrated} />
            </div>
            <ManualControlCard
              machine={machine}
              onGoHome={handleGoHome}
              onJog={handleJog}
              onPenDown={handlePenDown}
              onPenUp={handlePenUp}
              onTestStepperHoldPolicy={handleTestStepperHoldPolicy}
              onToggleYLoop={handleToggleYLoop}
            />
            <div data-step-anchor="run">
              <RunControls machine={machine} onPause={() => void handlePause()} onResume={() => void handleResume()} onRun={handleRun} onStop={handleStop} runReady={runReady} runStarting={busy.running} />
            </div>
          </div>
        }
        rightRail={
          <div className="rail-stack">
            <div data-step-anchor="prepare">
              <ImageImportCard
                disabled={!imageFile || busy.analyzing}
                hasAnalysis={Boolean(analysis)}
                imagePreviewUrl={imagePreviewUrl}
                onAnalyze={handleAnalyze}
                onFileChange={handleFileChange}
              />
            </div>
            <ColorPickerPanel analysis={analysis} onToggle={toggleColor} selectedColors={selectedColors} />
            <div data-step-anchor="generate">
              <PenSettingsCard canGenerate={Boolean(imageFile && selectedColors.length) && !busy.generating} onGenerate={handleGenerate} />
            </div>
            <JobSummaryPanel generationDurationMs={generationDurationMs} summary={summary} />
            <CalibrationPatternPanel generating={busy.generating} onGenerate={() => void handleGenerateCalibrationPattern()} />
            <XAxisCalibrationPanel generating={busy.generating} onGenerate={() => void handleGenerateXAxisCalibrationPattern()} />
            <AdvancedDrawer activeTab={drawerTab} onTab={setDrawerTab} />
            {advancedOpen && drawerTab === 'gcode' ? <GcodePanel gcode={gcode} /> : null}
            {advancedOpen && drawerTab === 'logs' ? <LogsPanel logs={logs} /> : null}
          </div>
        }
        stepNav={<StepNav hasImage={Boolean(imageFile)} hasPreview={Boolean(preview.length)} machine={machine} onSelect={jumpTo} />}
        topBar={
          <TopStatusBar
            canStop={Boolean(machine?.connected)}
            currentKind={currentKind}
            elapsedLabel={formatClock(elapsedSeconds)}
            machine={machine}
            onStop={handleStop}
            progressPercent={progressPercent}
            remainingLabel={formatClock(remainingSeconds)}
          />
        }
        workspace={
          <PreviewWorkspace
            imagePreviewUrl={imagePreviewUrl}
            machine={machine}
            maskPreviewUrl={maskPreviewUrl}
            onPreviewMode={setPreviewMode}
            onProgressFilter={setProgressFilter}
            onShowCompare={setShowCompare}
            onShowTravel={setShowTravel}
            onViewPreset={setViewPreset}
            paths={preview}
            previewMode={previewMode}
            progressFilter={progressFilter}
            showCompare={showCompare}
            showTravel={showTravel}
            viewPreset={viewPreset}
          />
        }
      />

      <div className="toast-stack">
        {toasts.map((toast) => (
          <div key={toast.id} className={`toast ${toast.tone}`}>
            {toast.message}
          </div>
        ))}
      </div>
    </>
  )
}

function BootMessage({ title, detail }: { title: string; detail: string }) {
  return (
    <div className="boot-screen">
      <div className="boot-card">
        <div className="panel-kicker">React Frontend</div>
        <h1>{title}</h1>
        <p>{detail}</p>
      </div>
    </div>
  )
}

function servoPayload(settings: SettingsState, sValue: number) {
  return {
    s: sValue,
    servo_ramp_enabled: settings.servoRampEnabled,
    servo_ramp_step: settings.servoRampStep,
    servo_ramp_delay_ms: settings.servoRampDelayMs,
    pen_up_dwell_ms: settings.penUpDwellMs,
    pen_down_dwell_ms: settings.penDownDwellMs,
  }
}

function buildGenerateFormData(file: File, settings: SettingsState, selectedColors: string[]) {
  const formData = new FormData()
  formData.append('image', file)
  formData.append('selected_colors', JSON.stringify(selectedColors))
  formData.append('draw_feed', String(settings.drawFeed))
  formData.append('travel_feed', String(settings.travelFeed))
  formData.append('artwork_scale_percent', String(settings.artworkScalePercent))
  formData.append('placement_scale', String(settings.placementScale))
  formData.append('placement_offset_x', String(settings.placementOffsetX))
  formData.append('placement_offset_y', String(settings.placementOffsetY))
  formData.append('rotation_deg', String(settings.rotationDeg))
  formData.append('line_thickness_mm', String(settings.lineThicknessMm))
  formData.append('wall_count', String(settings.wallCount))
  formData.append('infill_density', String(settings.infillDensity))
  formData.append('custom_infill_spacing', settings.customInfillSpacingEnabled ? '1' : '0')
  if (settings.customInfillSpacingEnabled) {
    formData.append('infill_spacing_mm', String(settings.infillSpacingMm))
  }
  formData.append('infill_angle_deg', String(settings.infillAngleDeg))
  formData.append('fill_strategy', settings.fillStrategy)
  formData.append('alternate_fill_angle_deg', String(settings.alternateFillAngleDeg))
  formData.append('sample_step_deg', String(settings.sampleStepDeg))
  formData.append('margin_percent', String(settings.marginPercent))
  formData.append('min_fill_area_mm2', String(settings.minFillAreaMm2))
  formData.append('min_fill_width_mm', String(settings.minFillWidthMm))
  formData.append('simplify_tolerance_mm', String(settings.simplifyToleranceMm))
  formData.append('min_segment_length_mm', String(settings.minSegmentLengthMm))
  formData.append('thin_detail_min_area_mm2', String(settings.thinDetailMinAreaMm2))
  formData.append('thin_detail_simplify_mm', String(settings.thinDetailSimplifyMm))
  formData.append('color_tolerance', String(settings.colorTolerance))
  formData.append('min_component_area_px', String(settings.minComponentAreaPx))
  formData.append('min_region_area_px', String(settings.minRegionAreaPx))
  formData.append('mask_open_radius_px', String(settings.maskOpenRadiusPx))
  formData.append('mask_close_radius_px', String(settings.maskCloseRadiusPx))
  formData.append('region_simplify_px', String(settings.regionSimplifyPx))
  formData.append('fit_mode', settings.fitMode)
  formData.append('invert_y', settings.invertY ? '1' : '0')
  formData.append('include_comments', settings.includeComments ? '1' : '0')
  formData.append('outline_after_fill', settings.outlineAfterFill ? '1' : '0')
  formData.append('remove_duplicate_paths', settings.removeDuplicatePaths ? '1' : '0')
  formData.append('thin_detail_mode', settings.thinDetailMode ? '1' : '0')
  formData.append('thin_detail_overlap', settings.thinDetailOverlap ? '1' : '0')
  formData.append('allow_pen_down_infill_connectors', settings.allowPenDownInfillConnectors ? '1' : '0')
  formData.append('pen_up_s', String(settings.penUpS))
  formData.append('pen_down_s', String(settings.penDownS))
  formData.append('pen_up_dwell_ms', String(settings.penUpDwellMs))
  formData.append('pen_down_dwell_ms', String(settings.penDownDwellMs))
  formData.append('servo_ramp_enabled', settings.servoRampEnabled ? '1' : '0')
  formData.append('servo_ramp_step', String(settings.servoRampStep))
  formData.append('servo_ramp_delay_ms', String(settings.servoRampDelayMs))
  return formData
}

function normalizeGenerateSettings(settings: SettingsState): SettingsState {
  const lineThicknessMm = Number(settings.lineThicknessMm)
  if (!Number.isFinite(lineThicknessMm)) {
    throw new Error('Missing required slicer setting: pen.line_thickness_mm')
  }
  const artworkScalePercent = clampArtworkScalePercent(settings.artworkScalePercent)
  return {
    ...settings,
    lineThicknessMm,
    artworkScalePercent,
    infillSpacingMm: settings.customInfillSpacingEnabled ? Number(settings.infillSpacingMm) : lineThicknessMm,
  }
}

function clampArtworkScalePercent(value: number) {
  if (!Number.isFinite(value)) return 100
  return Math.min(200, Math.max(10, Math.round(value)))
}

function normalizeEffectiveSettings(
  effectiveSettings: {
    line_thickness_mm: number
    infill_spacing_mm: number
    custom_infill_spacing: boolean
    wall_count: number
    fill_density: number
  } | undefined,
  settings: SettingsState,
) {
  const lineThicknessMm = Number(settings.lineThicknessMm)
  const infillSpacingMm = settings.customInfillSpacingEnabled ? Number(settings.infillSpacingMm) : lineThicknessMm
  return {
    line_thickness_mm:
      effectiveSettings && Number.isFinite(effectiveSettings.line_thickness_mm)
        ? effectiveSettings.line_thickness_mm
        : lineThicknessMm,
    infill_spacing_mm:
      effectiveSettings && Number.isFinite(effectiveSettings.infill_spacing_mm)
        ? effectiveSettings.infill_spacing_mm
        : infillSpacingMm,
    custom_infill_spacing:
      effectiveSettings?.custom_infill_spacing ?? settings.customInfillSpacingEnabled,
    wall_count:
      effectiveSettings && Number.isFinite(effectiveSettings.wall_count)
        ? effectiveSettings.wall_count
        : settings.wallCount,
    fill_density:
      effectiveSettings && Number.isFinite(effectiveSettings.fill_density)
        ? effectiveSettings.fill_density
        : settings.infillDensity,
  }
}

function sanitizePreviewPaths(paths: Array<Record<string, unknown>>): PreviewPath[] {
  return (paths ?? [])
    .map((path, index) => {
      const rawPoints = Array.isArray(path.points) ? path.points : []
      const points = rawPoints
        .map((point) => ({
          x: Number((point as { x?: unknown }).x),
          y: Number((point as { y?: unknown }).y),
        }))
        .filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y))
      return {
        ...path,
        id: typeof path.id === 'string' && path.id ? path.id : `preview-${index + 1}`,
        kind: typeof path.kind === 'string' ? path.kind : 'unknown',
        closed: Boolean(path.closed),
        points,
        gcode_start_line: typeof path.gcode_start_line === 'number' ? path.gcode_start_line : null,
        gcode_end_line: typeof path.gcode_end_line === 'number' ? path.gcode_end_line : null,
      }
    })
    .filter((path) => path.points.length >= 2)
}

function getElapsedSeconds(machine: MachineState | null) {
  return Math.max(0, machine?.job_elapsed_seconds ?? 0)
}

function getRemainingSeconds(
  machine: MachineState | null,
  summary: JobSummary | null,
  progressPercent: number,
  elapsedSeconds: number,
) {
  if (!machine) return 0
  if (machine.job_estimated_remaining_seconds != null) {
    return Math.max(0, machine.job_estimated_remaining_seconds)
  }
  if (isTerminalJobState(machine)) return 0
  if (!summary?.estimated_runtime_seconds) return 0
  if (!machine.running && !machine.paused) return 0
  if (progressPercent >= 100) return 0
  if (!progressPercent) return Math.max(0, summary.estimated_runtime_seconds - elapsedSeconds)
  const estimatedByProgress = elapsedSeconds * ((100 - progressPercent) / progressPercent)
  return Math.max(0, estimatedByProgress)
}

function isTerminalJobState(machine: MachineState | null) {
  const jobState = machine?.job_state?.toLowerCase()
  return jobState === 'completed' || jobState === 'stopped' || jobState === 'aborted' || jobState === 'error' || jobState === 'failed'
}

function formatClock(totalSeconds: number) {
  const seconds = Math.max(0, Math.round(totalSeconds))
  const hours = Math.floor(seconds / 3600)
  const minutes = Math.floor((seconds % 3600) / 60)
  const remainder = seconds % 60
  return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(remainder).padStart(2, '0')}`
}

function formatKind(kind: string) {
  return kind
    .replace(/^fill-/, '')
    .replace(/-/g, ' ')
    .replace(/\b\w/g, (char) => char.toUpperCase())
}

export default App
