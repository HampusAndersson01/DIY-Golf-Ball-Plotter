import { create } from 'zustand'

import type { AppConfig, AppDefaults, ImageAnalysis, JobSummary, MachineState, PreviewPath } from '../api/types'

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
  placementScale: number
  placementOffsetX: number
  placementOffsetY: number
  rotationDeg: number
  lineThicknessMm: number
  wallCount: number
  infillDensity: number
  infillSpacingMm: number
  infillAngleDeg: number
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
  setPreviewPayload: (payload: { preview: PreviewPath[]; maskPreviewUrl: string | null; gcode: string[]; summary: JobSummary | null }) => void
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
}

function buildSettings(defaults: AppDefaults): SettingsState {
  return {
    xMaxFeed: defaults.xMaxFeed,
    yMaxFeed: defaults.yMaxFeed,
    xAcceleration: defaults.xAcceleration,
    yAcceleration: defaults.yAcceleration,
    xJog: 1,
    yJog: 1,
    drawFeed: defaults.drawFeed,
    travelFeed: defaults.travelFeed,
    placementScale: 100,
    placementOffsetX: 0,
    placementOffsetY: 0,
    rotationDeg: defaults.rotationDeg,
    lineThicknessMm: defaults.lineThicknessMm,
    wallCount: defaults.wallCount,
    infillDensity: defaults.infillDensity,
    infillSpacingMm: defaults.infillSpacingMm,
    infillAngleDeg: defaults.infillAngleDeg,
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
  }),
  setAnalysis: (analysis) => set({ analysis, selectedColors: [] }),
  toggleColor: (hex) => set((state) => ({
    selectedColors: state.selectedColors.includes(hex)
      ? state.selectedColors.filter((entry) => entry !== hex)
      : [...state.selectedColors, hex],
  })),
  setPreviewPayload: ({ preview, maskPreviewUrl, gcode, summary }) => set({
    preview,
    maskPreviewUrl,
    gcode,
    summary,
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
  updateSetting: (key, value) => set((state) => ({
    settings: state.settings
      ? {
          ...state.settings,
          [key]: value,
        }
      : null,
  })),
}))
