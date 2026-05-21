import { create } from 'zustand'

import type { AppConfig, AppDefaults, CalibrationPattern, ImageAnalysis, JobSummary, MachineState, PreviewPath, XAxisCalibrationPattern } from '../api/types'

export type DrawerTab = 'advanced' | 'gcode' | 'logs'
export type PreviewMode = '2d' | '3d'
export type ProgressFilter = 'all' | 'progress'
export type ViewPreset = 'printer' | 'front'

export type SettingsState = {
  xMaxFeed: number
  yMaxFeed: number
  xAcceleration: number
  yAcceleration: number
  xJog: number
  yJog: number
  drawFeed: number
  travelFeed: number
  artworkScalePercent: number
  placementScale: number
  placementOffsetX: number
  placementOffsetY: number
  rotationDeg: number
  lineThicknessMm: number
  wallCount: number
  infillDensity: number
  infillSpacingMm: number
  customInfillSpacingEnabled: boolean
  infillAngleDeg: number
  fillStrategy: 'horizontal_scanline' | 'rotated_scanline' | 'adaptive_angle' | 'crosshatch'
  alternateFillAngleDeg: number
  sampleStepDeg: number
  marginPercent: number
  minFillAreaMm2: number
  minFillWidthMm: number
  simplifyToleranceMm: number
  minSegmentLengthMm: number
  thinDetailMinAreaMm2: number
  thinDetailSimplifyMm: number
  colorTolerance: number
  minComponentAreaPx: number
  minRegionAreaPx: number
  maskOpenRadiusPx: number
  maskCloseRadiusPx: number
  regionSimplifyPx: number
  maxColors: number
  simplifyColors: boolean
  fitMode: 'contain' | 'stretch'
  invertY: boolean
  includeComments: boolean
  outlineAfterFill: boolean
  removeDuplicatePaths: boolean
  thinDetailMode: boolean
  thinDetailOverlap: boolean
  allowPenDownInfillConnectors: boolean
  penUpS: number
  penDownS: number
  penUpDwellMs: number
  penDownDwellMs: number
  servoRampEnabled: boolean
  servoRampStep: number
  servoRampDelayMs: number
  streamingMode: 'buffered' | 'sync'
  yLoopDistance: number
  yLoopFeedrate: number
  yLoopDwellSec: number
  rawCommand: string
}

type Toast = {
  id: number
  tone: 'info' | 'success' | 'error'
  message: string
}

type BusyState = {
  bootstrapping: boolean
  connecting: boolean
  analyzing: boolean
  generating: boolean
  calibrating: boolean
  running: boolean
}

export type CalibrationMeasurement = {
  actualWidthMm: string
  actualHeightMm: string
  skipped: boolean
}

export type XAxisCalibrationMeasurement = {
  actualArcMm: string
}

type AppStore = {
  config: AppConfig | null
  settings: SettingsState | null
  machine: MachineState | null
  imageFile: File | null
  imagePreviewUrl: string | null
  analysis: ImageAnalysis | null
  selectedColors: string[]
  preview: PreviewPath[]
  maskPreviewUrl: string | null
  gcode: string[]
  summary: JobSummary | null
  calibrationPattern: CalibrationPattern | null
  calibrationMeasurements: Record<string, CalibrationMeasurement>
  xAxisCalibrationPattern: XAxisCalibrationPattern | null
  xAxisCalibrationMeasurements: Record<string, XAxisCalibrationMeasurement>
  logs: string[]
  toasts: Toast[]
  previewMode: PreviewMode
  progressFilter: ProgressFilter
  showTravel: boolean
  showCompare: boolean
  drawerTab: DrawerTab
  advancedOpen: boolean
  viewPreset: ViewPreset
  busy: BusyState
  initialize: (config: AppConfig) => void
  setMachine: (machine: MachineState) => void
  setImageFile: (file: File | null, previewUrl: string | null) => void
  setAnalysis: (analysis: ImageAnalysis | null) => void
  toggleColor: (hex: string) => void
  setPreviewPayload: (payload: { preview: PreviewPath[]; maskPreviewUrl: string | null; gcode: string[]; summary: JobSummary | null; calibrationPattern?: CalibrationPattern | null; xAxisCalibrationPattern?: XAxisCalibrationPattern | null }) => void
  setPreviewMode: (mode: PreviewMode) => void
  setProgressFilter: (filter: ProgressFilter) => void
  setShowTravel: (show: boolean) => void
  setShowCompare: (show: boolean) => void
  setDrawerTab: (tab: DrawerTab) => void
  setAdvancedOpen: (open: boolean) => void
  setViewPreset: (preset: ViewPreset) => void
  setBusy: (key: keyof BusyState, value: boolean) => void
  appendLog: (message: string) => void
  pushToast: (message: string, tone?: Toast['tone']) => void
  dismissToast: (id: number) => void
  updateSetting: <K extends keyof SettingsState>(key: K, value: SettingsState[K]) => void
  updateCalibrationMeasurement: (squareId: string, key: 'actualWidthMm' | 'actualHeightMm', value: string) => void
  setCalibrationSkipped: (squareId: string, skipped: boolean) => void
  updateXAxisCalibrationMeasurement: (measurementId: string, value: string) => void
}

function buildCalibrationMeasurements(pattern: CalibrationPattern | null): Record<string, CalibrationMeasurement> {
  if (!pattern) return {}
  return Object.fromEntries(
    pattern.squares.map((square) => [
      square.id,
      {
        actualWidthMm: '',
        actualHeightMm: '',
        skipped: false,
      },
    ]),
  )
}

function buildXAxisCalibrationMeasurements(pattern: XAxisCalibrationPattern | null): Record<string, XAxisCalibrationMeasurement> {
  if (!pattern) return {}
  const measurements: Record<string, XAxisCalibrationMeasurement> = {}
  for (let index = 1; index < pattern.ticks.length; index += 1) {
    const previousTick = pattern.ticks[index - 1]
    const currentTick = pattern.ticks[index]
    measurements[`${previousTick.id}_to_${currentTick.id}`] = { actualArcMm: '' }
  }
  measurements.overlap_error = { actualArcMm: '' }
  return measurements
}

function buildSettings(defaults: AppDefaults): SettingsState {
  const lineThicknessMm = defaults.lineThicknessMm
  return {
    xMaxFeed: defaults.xMaxFeed,
    yMaxFeed: defaults.yMaxFeed,
    xAcceleration: defaults.xAcceleration,
    yAcceleration: defaults.yAcceleration,
    xJog: 1,
    yJog: 1,
    drawFeed: defaults.drawFeed,
    travelFeed: defaults.travelFeed,
    artworkScalePercent: defaults.artworkScalePercent,
    placementScale: 100,
    placementOffsetX: 0,
    placementOffsetY: 0,
    rotationDeg: defaults.rotationDeg,
    lineThicknessMm,
    wallCount: defaults.wallCount,
    infillDensity: defaults.infillDensity,
    infillSpacingMm: lineThicknessMm,
    customInfillSpacingEnabled: defaults.customInfillSpacingEnabled ?? false,
    infillAngleDeg: defaults.infillAngleDeg,
    fillStrategy: defaults.fillStrategy ?? 'adaptive_angle',
    alternateFillAngleDeg: defaults.alternateFillAngleDeg ?? -45,
    sampleStepDeg: defaults.sampleStepDeg,
    marginPercent: defaults.marginPercent,
    minFillAreaMm2: defaults.minFillAreaMm2,
    minFillWidthMm: defaults.minFillWidthMm,
    simplifyToleranceMm: defaults.simplifyToleranceMm,
    minSegmentLengthMm: defaults.minSegmentLengthMm,
    thinDetailMinAreaMm2: defaults.thinDetailMinAreaMm2,
    thinDetailSimplifyMm: defaults.thinDetailSimplifyMm,
    colorTolerance: defaults.rasterColorTolerance,
    minComponentAreaPx: defaults.rasterMinComponentAreaPx,
    minRegionAreaPx: defaults.rasterMinRegionAreaPx,
    maskOpenRadiusPx: defaults.rasterMaskOpenRadiusPx,
    maskCloseRadiusPx: defaults.rasterMaskCloseRadiusPx,
    regionSimplifyPx: defaults.rasterRegionSimplifyPx,
    maxColors: defaults.rasterMaxColors,
    simplifyColors: true,
    fitMode: 'contain',
    invertY: true,
    includeComments: true,
    outlineAfterFill: defaults.outlineAfterFill,
    removeDuplicatePaths: defaults.removeDuplicatePaths,
    thinDetailMode: defaults.thinDetailMode,
    thinDetailOverlap: defaults.thinDetailOverlap,
    allowPenDownInfillConnectors: defaults.allowPenDownInfillConnectors,
    penUpS: defaults.penUpS,
    penDownS: defaults.penDownS,
    penUpDwellMs: defaults.penUpDwellMs,
    penDownDwellMs: defaults.penDownDwellMs,
    servoRampEnabled: defaults.servoRampEnabled,
    servoRampStep: defaults.servoRampStep,
    servoRampDelayMs: defaults.servoRampDelayMs,
    streamingMode: defaults.streamingMode ?? 'buffered',
    yLoopDistance: defaults.yLoopDistance ?? 10,
    yLoopFeedrate: defaults.yLoopFeedrate ?? defaults.drawFeed,
    yLoopDwellSec: defaults.yLoopDwellSec ?? 0.25,
    rawCommand: `M3 S${defaults.penUpS}`,
  }
}

export const useAppStore = create<AppStore>((set) => ({
  config: null,
  settings: null,
  machine: null,
  imageFile: null,
  imagePreviewUrl: null,
  analysis: null,
  selectedColors: [],
  preview: [],
  maskPreviewUrl: null,
  gcode: [],
  summary: null,
  calibrationPattern: null,
  calibrationMeasurements: {},
  xAxisCalibrationPattern: null,
  xAxisCalibrationMeasurements: {},
  logs: [],
  toasts: [],
  previewMode: '2d',
  progressFilter: 'all',
  showTravel: true,
  showCompare: false,
  drawerTab: 'advanced',
  advancedOpen: false,
  viewPreset: 'printer',
  busy: {
    bootstrapping: true,
    connecting: false,
    analyzing: false,
    generating: false,
    calibrating: false,
    running: false,
  },
  initialize: (config) => set({
    config,
    settings: buildSettings(config.defaults),
    busy: {
      bootstrapping: false,
      connecting: false,
      analyzing: false,
      generating: false,
      calibrating: false,
      running: false,
    },
  }),
  setMachine: (machine) => set((state) => ({
    machine,
    summary: state.summary ?? machine.last_summary,
  })),
  setImageFile: (file, previewUrl) => set({
    imageFile: file,
    imagePreviewUrl: previewUrl,
    analysis: null,
    selectedColors: [],
    preview: [],
    maskPreviewUrl: null,
    gcode: [],
    summary: null,
    calibrationPattern: null,
    calibrationMeasurements: {},
    xAxisCalibrationPattern: null,
    xAxisCalibrationMeasurements: {},
  }),
  setAnalysis: (analysis) => set({ analysis, selectedColors: [] }),
  toggleColor: (hex) => set((state) => ({
    selectedColors: state.selectedColors.includes(hex)
      ? state.selectedColors.filter((entry) => entry !== hex)
      : [...state.selectedColors, hex],
  })),
  setPreviewPayload: ({ preview, maskPreviewUrl, gcode, summary, calibrationPattern = null, xAxisCalibrationPattern = null }) => set({
    preview,
    maskPreviewUrl,
    gcode,
    summary,
    calibrationPattern,
    calibrationMeasurements: buildCalibrationMeasurements(calibrationPattern),
    xAxisCalibrationPattern,
    xAxisCalibrationMeasurements: buildXAxisCalibrationMeasurements(xAxisCalibrationPattern),
    drawerTab: 'advanced',
  }),
  setPreviewMode: (previewMode) => set({ previewMode }),
  setProgressFilter: (progressFilter) => set({ progressFilter }),
  setShowTravel: (showTravel) => set({ showTravel }),
  setShowCompare: (showCompare) => set({ showCompare }),
  setDrawerTab: (drawerTab) => set({ drawerTab }),
  setAdvancedOpen: (advancedOpen) => set({ advancedOpen }),
  setViewPreset: (viewPreset) => set({ viewPreset }),
  setBusy: (key, value) => set((state) => ({
    busy: {
      ...state.busy,
      [key]: value,
    },
  })),
  appendLog: (message) => set((state) => ({
    logs: [...state.logs.slice(-399), `${new Date().toLocaleTimeString()}  ${message}`],
  })),
  pushToast: (message, tone = 'info') => set((state) => ({
    toasts: [...state.toasts, { id: Date.now() + state.toasts.length, tone, message }],
  })),
  dismissToast: (id) => set((state) => ({
    toasts: state.toasts.filter((toast) => toast.id !== id),
  })),
  updateSetting: (key, value) => set((state) => {
    if (!state.settings) return { settings: null }

    const nextSettings: SettingsState = {
      ...state.settings,
      [key]: value,
    }

    if (key === 'lineThicknessMm' && typeof value === 'number') {
      if (!nextSettings.customInfillSpacingEnabled) {
        nextSettings.infillSpacingMm = value
      }
    }

    if (key === 'customInfillSpacingEnabled' && value === false) {
      nextSettings.infillSpacingMm = nextSettings.lineThicknessMm
    }

    return { settings: nextSettings }
  }),
  updateCalibrationMeasurement: (squareId, key, value) => set((state) => ({
    calibrationMeasurements: {
      ...state.calibrationMeasurements,
      [squareId]: {
        ...state.calibrationMeasurements[squareId],
        actualWidthMm: state.calibrationMeasurements[squareId]?.actualWidthMm ?? '',
        actualHeightMm: state.calibrationMeasurements[squareId]?.actualHeightMm ?? '',
        skipped: state.calibrationMeasurements[squareId]?.skipped ?? false,
        [key]: value,
      },
    },
  })),
  setCalibrationSkipped: (squareId, skipped) => set((state) => ({
    calibrationMeasurements: {
      ...state.calibrationMeasurements,
      [squareId]: {
        ...state.calibrationMeasurements[squareId],
        actualWidthMm: state.calibrationMeasurements[squareId]?.actualWidthMm ?? '',
        actualHeightMm: state.calibrationMeasurements[squareId]?.actualHeightMm ?? '',
        skipped,
      },
    },
  })),
  updateXAxisCalibrationMeasurement: (measurementId, value) => set((state) => ({
    xAxisCalibrationMeasurements: {
      ...state.xAxisCalibrationMeasurements,
      [measurementId]: { actualArcMm: value },
    },
  })),
}))
