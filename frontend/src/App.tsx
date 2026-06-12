import { startTransition, useCallback, useEffect, useEffectEvent, useMemo, useRef, useState } from 'react'
import type { ChangeEvent } from 'react'

import { analyzeImage, fetchBootstrap, fetchState, generateDiagnosticGcode, generateImageGcode } from './api/client'
import { CalibrationPatternPanel } from './components/calibration/CalibrationPatternPanel'
import { XAxisCalibrationPanel } from './components/calibration/XAxisCalibrationPanel'
import { StepNav } from './components/layout/StepNav'
import { PrintSetupPanel } from './components/image/PrintSetupPanel'
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
import { GrblWebSerialService } from './services/grblWebSerial'
import type { SettingsState } from './store/appStore'
import { useAppStore } from './store/appStore'
import type { ComponentType } from 'react'
import { MdChevronLeft, MdChevronRight, MdPrint, MdSettingsApplications, MdUsb } from 'react-icons/md'

type SidebarCategory = 'control' | 'output' | 'advanced'

type SidebarIcon = ComponentType<{ 'aria-hidden'?: boolean }>

const SIDEBAR_CATEGORIES: Array<{ id: SidebarCategory; label: string; icon: SidebarIcon }> = [
  { id: 'control', label: 'Control', icon: () => <MdUsb aria-hidden="true" /> },
  { id: 'output', label: 'Output', icon: () => <MdPrint aria-hidden="true" /> },
  { id: 'advanced', label: 'Advanced', icon: () => <MdSettingsApplications aria-hidden="true" /> },
]

const grblSerial = new GrblWebSerialService()

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
  const config = useAppStore((state) => state.config)
  const machine = useAppStore((state) => state.machine)
  const settings = useAppStore((state) => state.settings)
  const imageFile = useAppStore((state) => state.imageFile)
  const imagePreviewUrl = useAppStore((state) => state.imagePreviewUrl)
  const analysis = useAppStore((state) => state.analysis)
  const selectedColors = useAppStore((state) => state.selectedColors)
  const preview = useAppStore((state) => state.preview)
  const maskPreviewUrl = useAppStore((state) => state.maskPreviewUrl)
  const maskProjectionQuad = useAppStore((state) => state.maskProjectionQuad)
  const maskProjectedPreview = useAppStore((state) => state.maskProjectedPreview)
  const gcode = useAppStore((state) => state.gcode)
  const summary = useAppStore((state) => state.summary)
  const stageTimings = useAppStore((state) => state.stageTimings)
  const logs = useAppStore((state) => state.logs)
  const toasts = useAppStore((state) => state.toasts)
  const previewMode = useAppStore((state) => state.previewMode)
  const progressFilter = useAppStore((state) => state.progressFilter)
  const showTravel = useAppStore((state) => state.showTravel)
  const showPenWidth = useAppStore((state) => state.showPenWidth)
  const showMask = useAppStore((state) => state.showMask)
  const drawerTab = useAppStore((state) => state.drawerTab)
  const advancedOpen = useAppStore((state) => state.advancedOpen)
  const viewPreset = useAppStore((state) => state.viewPreset)
  const busy = useAppStore((state) => state.busy)
  const setMachine = useAppStore((state) => state.setMachine)
  const setSerialPort = useAppStore((state) => state.setSerialPort)
  const setImageFile = useAppStore((state) => state.setImageFile)
  const setAnalysis = useAppStore((state) => state.setAnalysis)
  const toggleColor = useAppStore((state) => state.toggleColor)
  const setPreviewPayload = useAppStore((state) => state.setPreviewPayload)
  const setPreviewMode = useAppStore((state) => state.setPreviewMode)
  const setProgressFilter = useAppStore((state) => state.setProgressFilter)
  const setShowTravel = useAppStore((state) => state.setShowTravel)
  const setShowPenWidth = useAppStore((state) => state.setShowPenWidth)
  const setShowMask = useAppStore((state) => state.setShowMask)
  const setDrawerTab = useAppStore((state) => state.setDrawerTab)
  const setViewPreset = useAppStore((state) => state.setViewPreset)
  const setBusy = useAppStore((state) => state.setBusy)
  const appendLog = useAppStore((state) => state.appendLog)
  const pushToast = useAppStore((state) => state.pushToast)
  const dismissToast = useAppStore((state) => state.dismissToast)
  const [generationDurationMs, setGenerationDurationMs] = useState<number | null>(null)
  const [activeSidebarCategory, setActiveSidebarCategory] = useState<SidebarCategory>('output')
  const [inspectorCollapsed, setInspectorCollapsed] = useState(true)
  const [previewZoomLabel, setPreviewZoomLabel] = useState('100%')
  const restoredPersistedPreviewRef = useRef(false)
  const lastSeenPlacementPreviewKeyRef = useRef<string | null>(null)
  const pendingPlacementPreviewKeyRef = useRef<string | null>(null)
  const lastRunUiSyncAtRef = useRef(0)

  const readySettings = settings
  const progressPercent = getProgressPercent(machine)
  const runReady = Boolean(machine?.connected && machine?.calibrated && gcode.length && !machine?.y_loop_test?.enabled && !machine?.running && !busy.running)
  const runLockReason = !machine?.connected
    ? 'Connect machine first'
    : !machine?.calibrated
      ? 'Calibration pending'
      : !gcode.length
        ? 'Generate a job first'
        : machine?.y_loop_test?.enabled
          ? 'Stop Y-loop test before run'
          : machine?.running
            ? 'Job is currently running'
            : busy.running
            ? 'Job startup in progress'
            : null
  const currentSettings = useMemo(() => readySettings, [readySettings])
  const elapsedSeconds = getElapsedSeconds(machine)
  const remainingSeconds = getRemainingSeconds(machine, summary, progressPercent, elapsedSeconds)

  const previewPathByLine = useMemo(() => {
    return preview
      .filter((path) => path.gcode_start_line != null && path.gcode_end_line != null)
      .sort((a, b) => (a.gcode_start_line ?? 0) - (b.gcode_start_line ?? 0))
  }, [preview])

  const onKeyboardPause = useEffectEvent(async () => {
    if (!currentSettings) return
    if (machine?.paused) await handleResume()
    else await handlePause()
  })

  const hydrateGeneratedPreviewFromMachineOnce = useCallback((nextState: MachineState) => {
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
      maskProjectionQuad: null,
      maskProjectedPreview: [],
      gcode: hydratedGcode,
      summary: hydratedSummary,
      calibrationPattern: null,
      xAxisCalibrationPattern: null,
    })
  }, [setPreviewPayload])

  const decorateMachineState = useCallback((machineState: MachineState) => {
    const activePath = findPreviewPathForLine(previewPathByLine, machineState.current_gcode_line || 0)
    if (!activePath) {
      return {
        ...machineState,
        current_path_id: null,
        current_path_kind: null,
        current_preview_point_index: 0,
      }
    }
    return {
      ...machineState,
      current_path_id: activePath.id ?? null,
      current_path_kind: activePath.kind ?? null,
      current_preview_point_index: getPreviewPointIndexForLine(activePath, machineState.current_gcode_line || 0),
    }
  }, [previewPathByLine])

  const syncBrowserMachineState = useCallback(() => {
    const nextMachine = decorateMachineState(grblSerial.getMachineState())
    startTransition(() => {
      setMachine(nextMachine)
      setSerialPort(grblSerial.getPort())
    })
  }, [decorateMachineState, setMachine, setSerialPort])

  useEffect(() => {
    let active = true
    const load = async () => {
      try {
        const nextState = await fetchState()
        if (!active) return
        hydrateGeneratedPreviewFromMachineOnce(nextState)
        syncBrowserMachineState()
      } catch (error) {
        if (!active) return
        appendLog(`State bootstrap failed: ${String(error)}`)
      }
    }
    void load()
    return () => {
      active = false
    }
  }, [appendLog, hydrateGeneratedPreviewFromMachineOnce, syncBrowserMachineState])

  useEffect(() => {
    let active = true
    const inspectApprovedPorts = async () => {
      try {
        const ports = await grblSerial.getPreviouslyApprovedPorts()
        if (!active || !ports.length) return
        appendLog(`Browser already has permission for ${ports.length} serial port${ports.length === 1 ? '' : 's'}. Click Connect plotter to choose one.`)
      } catch (error) {
        if (!active) return
        appendLog(`Approved-port check skipped: ${String(error)}`)
      }
    }
    void inspectApprovedPorts()
    return () => {
      active = false
    }
  }, [appendLog])

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
      if (event.code === 'Space' && machine?.running) {
        event.preventDefault()
        void onKeyboardPause()
      }
    }
    window.addEventListener('keydown', onKeyDown)
    return () => window.removeEventListener('keydown', onKeyDown)
  }, [machine?.running])

  useEffect(() => {
    if (!machine?.running && !machine?.paused) return
    const timer = window.setInterval(() => {
      syncBrowserMachineState()
    }, 250)
    return () => window.clearInterval(timer)
  }, [machine?.paused, machine?.running, syncBrowserMachineState])

  const placementPreviewKey = readySettings
    ? [
      readySettings.originAnchor,
      readySettings.originOffsetXmm,
      readySettings.originOffsetYmm,
    ].join('|')
    : 'uninitialized'

  const refreshPlacedPreview = useEffectEvent(() => {
    if (!readySettings) return
    void handleGenerate()
  })

  useEffect(() => {
    if (!readySettings) return
    const previousKey = lastSeenPlacementPreviewKeyRef.current
    lastSeenPlacementPreviewKeyRef.current = placementPreviewKey
    if (previousKey == null || previousKey === placementPreviewKey) return
    pendingPlacementPreviewKeyRef.current = placementPreviewKey
  }, [placementPreviewKey, readySettings])

  useEffect(() => {
    if (!readySettings) return
    if (pendingPlacementPreviewKeyRef.current !== placementPreviewKey) return
    if (!imageFile || !selectedColors.length || (!preview.length && !gcode.length) || busy.generating) return
    const timer = window.setTimeout(() => {
      pendingPlacementPreviewKeyRef.current = null
      refreshPlacedPreview()
    }, 250)
    return () => window.clearTimeout(timer)
  }, [busy.generating, gcode.length, imageFile, placementPreviewKey, preview.length, readySettings, selectedColors.length])

  if (!readySettings) {
    return <BootMessage title="Loading dashboard" detail="Preparing local operator state." />
  }
  if (!config) {
    return <BootMessage title="Dashboard bootstrap failed" detail="Missing frontend config." />
  }

  const settingsState = readySettings

  function refreshMachine() {
    syncBrowserMachineState()
  }

  function recordMachineResult(result: { command?: string; response?: string } | null, successMessage?: string) {
    if (result?.command) appendLog(`> ${result.command}`)
    if (result?.response) appendLog(`< ${result.response}`)
    if (successMessage) pushToast(successMessage, 'success')
    refreshMachine()
  }

  async function handleConnect() {
    setBusy('connecting', true)
    try {
      const port = await grblSerial.connect()
      setSerialPort(port)
      syncBrowserMachineState()
      pushToast('Plotter connected.', 'success')
      appendLog('Connect plotter opened the browser picker and connected at 115200 baud.')
    } catch (error) {
      pushToast(String(error), 'error')
      appendLog(`Connect failed: ${String(error)}`)
    } finally {
      setBusy('connecting', false)
    }
  }

  async function handleDisconnect() {
    try {
      await grblSerial.disconnect()
      syncBrowserMachineState()
      pushToast('Plotter disconnected.', 'success')
      appendLog('Serial port released.')
    } catch (error) {
      pushToast(String(error), 'error')
      appendLog(`Disconnect failed: ${String(error)}`)
    }
  }

  async function handleApplyConfig() {
    try {
      const result = await grblSerial.applyConfig(config!, settingsState)
      recordMachineResult(result, 'Machine settings applied.')
    } catch (error) {
      pushToast(String(error), 'error')
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
      lastSeenPlacementPreviewKeyRef.current = buildPlacementPreviewKey(normalizedSettings)
      pendingPlacementPreviewKeyRef.current = null
      setPreviewPayload({
        preview,
        maskPreviewUrl: payload.mask_preview,
        maskProjectionQuad: payload.mask_projection_quad ?? null,
        maskProjectedPreview: payload.mask_projected_preview ?? [],
        gcode: payload.gcode,
        summary: payload.summary,
        stageTimings: payload.stage_timings,
        calibrationPattern: payload.calibrationPattern ?? null,
        xAxisCalibrationPattern: payload.xAxisCalibrationPattern ?? null,
      })
      setGenerationDurationMs(performance.now() - startedAt)
      appendLog(`Generated ${payload.gcode.length} G-code lines across ${preview.length} preview paths.`)
      appendLog(
        `Effective slicer settings: pen ${effectiveSettings.line_thickness_mm.toFixed(2)} mm, infill spacing ${effectiveSettings.infill_spacing_mm.toFixed(2)} mm, custom spacing ${effectiveSettings.custom_infill_spacing ? 'on' : 'off'}.`,
      )
      pushToast('G-code generated.', 'success')
    } catch (error) {
      pushToast(String(error), 'error')
      appendLog(`Generate failed: ${String(error)}`)
    } finally {
      setBusy('generating', false)
    }
  }

  async function handlePenUp() {
    const result = await grblSerial.penUp(settingsState)
    recordMachineResult(result, 'Pen up.')
  }

  async function handlePenDown() {
    const result = await grblSerial.penDown(settingsState)
    recordMachineResult(result, 'Pen down.')
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
        maskProjectionQuad: null,
        maskProjectedPreview: [],
        gcode: payload.gcode,
        summary: payload.summary,
        stageTimings: payload.stage_timings,
        calibrationPattern: payload.calibrationPattern ?? null,
        xAxisCalibrationPattern: payload.xAxisCalibrationPattern ?? null,
      })
      appendLog(`Generated diagnostic pattern ${payload.calibrationPattern?.pattern ?? '3x3_squares'} with ${preview.length} preview paths.`)
      pushToast('Calibration test pattern generated.', 'success')
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
        maskProjectionQuad: null,
        maskProjectedPreview: [],
        gcode: payload.gcode,
        summary: payload.summary,
        stageTimings: payload.stage_timings,
        calibrationPattern: payload.calibrationPattern ?? null,
        xAxisCalibrationPattern: payload.xAxisCalibrationPattern ?? null,
      })
      appendLog(`Generated X rotary calibration pattern with ${preview.length} preview paths.`)
      pushToast('X rotary calibration pattern generated.', 'success')
    } catch (error) {
      pushToast(String(error), 'error')
      appendLog(`X rotary calibration generation failed: ${String(error)}`)
    } finally {
      setBusy('generating', false)
    }
  }

  async function handleGoHome() {
    const result = await grblSerial.goHome(settingsState)
    recordMachineResult(result, 'Returned to origin.')
  }

  async function handleJog(axis: 'X' | 'Y', degrees: number) {
    const result = await grblSerial.jog(axis, degrees, settingsState.travelFeed)
    recordMachineResult(result)
  }

  async function handleCalibrate() {
    if (!window.confirm('Confirm the pen is physically positioned at the center of the ball.')) return
    setBusy('calibrating', true)
    try {
      const result = await grblSerial.zeroAndMarkCalibrated()
      recordMachineResult(result, 'Origin locked.')
    } catch (error) {
      pushToast(String(error), 'error')
    } finally {
      setBusy('calibrating', false)
    }
  }

  async function handleClearCalibrated() {
    const result = await grblSerial.clearCalibrated()
    recordMachineResult(result, 'Calibration cleared.')
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
      const result = await grblSerial.applyStepperHoldPolicy()
      recordMachineResult(result, 'Release policy applied.')
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
    lastRunUiSyncAtRef.current = 0
    const runPromise = grblSerial.runGcode(gcode, {
      onProgress: () => {
        const now = performance.now()
        if (now - lastRunUiSyncAtRef.current < 120) {
          return
        }
        lastRunUiSyncAtRef.current = now
        syncBrowserMachineState()
      },
      streamingMode: settingsState.streamingMode,
    })

    syncBrowserMachineState()
    appendLog(`Run started: streaming ${gcode.length} G-code lines over Web Serial.`)
    pushToast('Job started.', 'success')
    setBusy('running', false)

    void runPromise
      .then((result) => {
        recordMachineResult(result, 'Job complete.')
      })
      .catch((error) => {
        syncBrowserMachineState()
        pushToast(String(error), 'error')
        appendLog(`Run failed: ${String(error)}`)
      })
  }

  async function handlePause() {
    const result = await grblSerial.pause()
    recordMachineResult(result, 'Pause requested.')
  }

  async function handleResume() {
    const result = await grblSerial.resume()
    recordMachineResult(result, 'Resume requested.')
  }

  async function handleStop() {
    const result = await grblSerial.stop()
    recordMachineResult(result, 'Stop requested.')
  }

  async function handleToggleYLoop() {
    try {
      const enabled = Boolean(machine?.y_loop_test?.enabled)
      if (enabled) {
        const result = await grblSerial.stopYLoop()
        recordMachineResult(result, 'Y loop test stopped.')
        return
      }
      const result = await grblSerial.startYLoop(settingsState)
      recordMachineResult(result, 'Y loop test started.')
    } catch (error) {
      pushToast(String(error), 'error')
      appendLog(`Y loop test failed: ${String(error)}`)
    }
  }

  async function handleFileChange(event: ChangeEvent<HTMLInputElement>) {
    const file = event.target.files?.[0] ?? null
    const previewUrl = file ? URL.createObjectURL(file) : null
    if (imagePreviewUrl) URL.revokeObjectURL(imagePreviewUrl)
    setImageFile(file, previewUrl)
    if (file) {
      void handleAnalyzeFile(file)
    }
  }

  async function handleAnalyzeFile(file: File) {
    setBusy('analyzing', true)
    try {
      const formData = new FormData()
      formData.append('image', file)
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

  function jumpTo(step: string) {
    const mapping: Record<string, SidebarCategory> = {
      connect: 'control',
      calibrate: 'control',
      prepare: 'output',
      generate: 'output',
      run: 'control',
    }
    setActiveSidebarCategory(mapping[step] ?? 'control')
  }

  const machineSetupPanel = (
    <div className="sidebar-stack">
      <section className="sidebar-panel sidebar-panel--surface">
        <div className="sidebar-panel__header">
          <div>
            <div className="panel-kicker">Machine setup</div>
            <h2>Connection, calibration, manual</h2>
          </div>
          <span className={`badge ${machine?.connected ? 'good' : 'muted'}`}>{machine?.connected ? 'Online' : 'Offline'}</span>
        </div>
        <div className="sidebar-panel__status-row">
          <div className={`status-pill ${machine?.connected ? 'good' : 'muted'}`}>
            <span>Connection</span>
            <strong>{machine?.connected ? 'Connected' : 'Disconnected'}</strong>
          </div>
          <div className={`status-pill ${machine?.calibrated ? 'good' : 'warn'}`}>
            <span>Calibration</span>
            <strong>{machine?.calibrated ? 'Locked' : 'Pending'}</strong>
          </div>
          <div className={`status-pill ${runReady ? 'good' : 'warn'}`}>
            <span>Job</span>
            <strong>{runReady ? 'Ready' : 'Locked'}</strong>
          </div>
        </div>
      </section>

      <MachineCard onApplyConfig={handleApplyConfig} onConnect={handleConnect} onDisconnect={handleDisconnect} />
      <CalibrationCard machine={machine} onCalibrate={handleCalibrate} onClear={handleClearCalibrated} />
      <ManualControlCard
        machine={machine}
        onGoHome={handleGoHome}
        onJog={handleJog}
        onPenDown={handlePenDown}
        onPenUp={handlePenUp}
        onTestStepperHoldPolicy={handleTestStepperHoldPolicy}
        onToggleYLoop={handleToggleYLoop}
      />
      <RunControls
        machine={machine}
        onPause={() => void handlePause()}
        onResume={() => void handleResume()}
        onRun={handleRun}
        onStop={handleStop}
        runReady={runReady}
        runStarting={busy.running}
      />

      <section className="sidebar-panel sidebar-panel--compact">
        <details className="details-panel">
          <summary>Calibration tests</summary>
          <div className="sidebar-panel__test-stack">
            <CalibrationPatternPanel generating={busy.generating} onGenerate={() => void handleGenerateCalibrationPattern()} />
            <XAxisCalibrationPanel generating={busy.generating} onGenerate={() => void handleGenerateXAxisCalibrationPattern()} />
          </div>
        </details>
      </section>
    </div>
  )

  const printSetupPanel = (
    <div className="sidebar-stack">
      <PrintSetupPanel
        analysis={analysis}
        canGenerate={Boolean(imageFile && selectedColors.length) && !busy.generating}
        imagePreviewUrl={imagePreviewUrl}
        onFileChange={handleFileChange}
        onGenerate={handleGenerate}
        onToggleColor={toggleColor}
        selectedColors={selectedColors}
      />
    </div>
  )

  const advancedPanel = (
    <div className="sidebar-stack">
      <section className="sidebar-panel sidebar-panel--surface">
        <div className="sidebar-panel__header">
          <div>
            <div className="panel-kicker">Advanced</div>
            <h2>Settings and diagnostics</h2>
          </div>
        </div>
        <AdvancedDrawer activeTab={drawerTab} onTab={setDrawerTab} />
      </section>
      {advancedOpen && drawerTab === 'gcode' ? <GcodePanel gcode={gcode} /> : null}
      {advancedOpen && drawerTab === 'logs' ? <LogsPanel logs={logs} /> : null}
    </div>
  )

  const sidebarPanel = activeSidebarCategory === 'output'
    ? printSetupPanel
    : activeSidebarCategory === 'advanced'
      ? advancedPanel
      : machineSetupPanel

  return (
    <>
      <div className={`plotter-dashboard ${inspectorCollapsed ? 'inspector-collapsed' : ''}`}>
        <header className="control-header">
          <div className="control-header__title">
            <h1>DIY Golf Ball Plotter</h1>
            <p>Raster G-code generated — calibrate before run</p>
          </div>

          <div className="control-header__status">
            <div className="header-progress-card" aria-label="Job progress">
              <div className="header-progress-card__top">
                <span>Progress</span>
                <strong>{progressPercent}%</strong>
              </div>
              <div className="header-progress-track" aria-hidden="true">
                <div className="header-progress-fill" style={{ width: `${progressPercent}%` }} />
              </div>
              <div className="header-progress-card__metrics">
                <div className="header-metric">
                  <span>Elapsed</span>
                  <strong>{formatClock(elapsedSeconds)}</strong>
                </div>
                <div className="header-metric">
                  <span>Remaining</span>
                  <strong>{formatClock(remainingSeconds)}</strong>
                </div>
              </div>
            </div>
          </div>

          <div className="control-header__actions">
            <button className="emergency-stop" disabled={!machine?.connected} onClick={handleStop} type="button">
              EMERGENCY STOP
            </button>

            <button
              aria-label={inspectorCollapsed ? 'Show details' : 'Hide details'}
              className="button subtle header-details-button"
              onClick={() => setInspectorCollapsed((value) => !value)}
              type="button"
            >
              <span>{inspectorCollapsed ? 'Show details' : 'Hide details'}</span>
              {inspectorCollapsed ? <MdChevronLeft aria-hidden="true" /> : <MdChevronRight aria-hidden="true" />}
            </button>
          </div>
        </header>

        <StepNav
          hasImage={Boolean(imageFile)}
          hasPreview={Boolean(preview.length)}
          machine={machine}
          onSelect={jumpTo}
          runLockReason={runLockReason}
          runReady={runReady}
        />

        <div className="dashboard-grid">
          <aside className="left-rail" aria-label="Control panel">
            <div className="sidebar-shell">
                <div className="sidebar-nav-header" aria-hidden>
                  <div className="tab-row" role="tablist" aria-label="Control categories">
                    {SIDEBAR_CATEGORIES.map((category) => (
                      <button
                        key={category.id}
                        className={`sidebar-nav-item ${activeSidebarCategory === category.id ? 'active' : ''}`}
                        onClick={() => setActiveSidebarCategory(category.id)}
                        title={category.label}
                        role="tab"
                        type="button"
                      >
                        <span className="sidebar-nav-item__icon">
                          <category.icon />
                        </span>
                        <span className="sidebar-nav-item__label">{category.label}</span>
                      </button>
                    ))}
                  </div>
                </div>

              <div className="sidebar-content" role="tabpanel">
                {sidebarPanel}
              </div>
            </div>
          </aside>

          <main className="workspace-panel">
            <PreviewWorkspace
              ballDiameterMm={config.ballDiameterMm}
              imagePreviewUrl={imagePreviewUrl}
              machine={machine}
              maskPreviewUrl={maskPreviewUrl}
              maskProjectionQuad={maskProjectionQuad}
              maskProjectedPreview={maskProjectedPreview}
              maxPrintXSpanDeg={config.defaults.maxPrintXSpanDeg}
              lineThicknessMm={settingsState.lineThicknessMm}
              onPreviewMode={setPreviewMode}
              onProgressFilter={setProgressFilter}
              onShowPenWidth={setShowPenWidth}
              onShowMask={setShowMask}
              onShowTravel={setShowTravel}
              onViewPreset={setViewPreset}
              paths={preview}
              previewMode={previewMode}
              progressFilter={progressFilter}
              showPenWidth={showPenWidth}
              showMask={showMask}
              showTravel={showTravel}
              onZoomChange={setPreviewZoomLabel}
              zoomLabel={previewZoomLabel}
              viewPreset={viewPreset}
            />
          </main>

          <aside className={`right-rail dashboard-inspector ${inspectorCollapsed ? 'collapsed' : ''}`}>
            <div className="inspector-shell">
              {!inspectorCollapsed ? (
                <div className="inspector-stack">
                  <section className="inspector-card inspector-card--image">
                    <div className="inspector-card__header">
                      <div>
                        <span className="panel-kicker">Input image</span>
                        <strong>{analysis ? `${analysis.width}x${analysis.height}px` : 'No image yet'}</strong>
                      </div>
                      <div className="inspector-icon">▣</div>
                    </div>
                    <div className="thumb-frame">
                      {imagePreviewUrl ? <img alt="Selected input" src={imagePreviewUrl} /> : <span>Image preview</span>}
                    </div>
                  </section>

                  <div className="inspector-card-grid">
                    <section className="inspector-card inspector-card--metric">
                      <span className="panel-kicker">G-code lines</span>
                      <strong>{summary?.gcode_line_count ?? gcode.length ?? '--'}</strong>
                    </section>
                    <section className="inspector-card inspector-card--metric">
                      <span className="panel-kicker">Estimated runtime</span>
                      <strong>{summary ? formatClock(summary.estimated_runtime_seconds) : '--'}</strong>
                    </section>
                    <section className="inspector-card inspector-card--metric">
                      <span className="panel-kicker">Pen lifts</span>
                      <strong>{summary?.pen_lift_count ?? '--'}</strong>
                    </section>
                    <section className="inspector-card inspector-card--metric">
                      <span className="panel-kicker">Readiness</span>
                      <strong>{machine?.calibrated ? 'Ready to Calibrate' : 'Calibration pending'}</strong>
                    </section>
                  </div>

                  <section className="inspector-card inspector-card--note">
                    <div className="inspector-card__header">
                      <span className="panel-kicker">Job status</span>
                      <span className={`badge ${runReady ? 'good' : 'warn'}`}>{runReady ? 'Ready' : 'Locked'}</span>
                    </div>
                    <p>{machine?.status ?? 'Idle'}{runLockReason ? ` · ${runLockReason}` : ''}</p>
                  </section>

                  <details className="details-panel inspector-details">
                    <summary>Detailed metrics</summary>
                    <JobSummaryPanel generationDurationMs={generationDurationMs} stageTimings={stageTimings} summary={summary} />
                  </details>
                </div>
              ) : null}
            </div>
          </aside>
        </div>
      </div>

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

function buildGenerateFormData(file: File, settings: SettingsState, selectedColors: string[]) {
  const formData = new FormData()
  formData.append('image', file)
  formData.append('selected_colors', JSON.stringify(selectedColors))
  formData.append('simplify_colors', settings.simplifyColors ? '1' : '0')
  formData.append('max_colors', String(settings.maxColors))
  formData.append('draw_feed', String(settings.drawFeed))
  formData.append('travel_feed', String(settings.travelFeed))
  formData.append('artwork_scale_percent', String(settings.artworkScalePercent))
  formData.append('origin_anchor', settings.originAnchor)
  formData.append('origin_offset_x_mm', String(settings.originOffsetXmm))
  formData.append('origin_offset_y_mm', String(settings.originOffsetYmm))
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
  formData.append('ignore_printable_x_span_limit', settings.ignorePrintableXSpanLimit ? '1' : '0')
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
    originOffsetXmm: normalizeFiniteNumber(settings.originOffsetXmm, 0),
    originOffsetYmm: normalizeFiniteNumber(settings.originOffsetYmm, 0),
    infillSpacingMm: settings.customInfillSpacingEnabled ? Number(settings.infillSpacingMm) : lineThicknessMm,
  }
}

function clampArtworkScalePercent(value: number) {
  if (!Number.isFinite(value)) return 100
  return Math.min(200, Math.max(10, Math.round(value)))
}

function normalizeFiniteNumber(value: number, fallback: number) {
  return Number.isFinite(value) ? value : fallback
}

function buildPlacementPreviewKey(settings: Pick<SettingsState, 'originAnchor' | 'originOffsetXmm' | 'originOffsetYmm'>) {
  return [settings.originAnchor, settings.originOffsetXmm, settings.originOffsetYmm].join('|')
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

function findPreviewPathForLine(paths: PreviewPath[], lineNumber: number) {
  return paths.find((path) => {
    const start = path.gcode_start_line
    const end = path.gcode_end_line
    return start != null && end != null && lineNumber >= start && lineNumber <= end
  }) ?? null
}

function getPreviewPointIndexForLine(path: PreviewPath, lineNumber: number) {
  const start = path.gcode_start_line
  const end = path.gcode_end_line
  if (start == null || end == null || end <= start || path.points.length <= 1) {
    return 0
  }
  const progress = Math.min(1, Math.max(0, (lineNumber - start) / Math.max(1, end - start)))
  return Math.min(path.points.length - 1, Math.max(0, Math.round(progress * (path.points.length - 1))))
}

export default App
