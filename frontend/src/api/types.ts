export type PreviewKind = 'fill-wall' | 'fill-infill' | 'detail-trace' | 'outline' | 'travel' | string

export type PreviewPoint = {
  x: number
  y: number
}

export type PreviewPath = {
  id?: string
  kind: PreviewKind
  closed: boolean
  points: PreviewPoint[]
  gcode_start_line: number | null
  gcode_end_line: number | null
}

export type CalibrationBbox = {
  minX: number
  minY: number
  maxX: number
  maxY: number
  width: number
  height: number
  centerX: number
  centerY: number
}

export type CalibrationSquare = {
  id: string
  label: string
  row: number
  col: number
  surfaceMmBbox: CalibrationBbox
  surfaceMmToolpathBbox: CalibrationBbox | null
  machineDegreeBbox: CalibrationBbox | null
  gcodeBbox: CalibrationBbox | null
  expectedSurfaceWidthMm: number
  expectedSurfaceHeightMm: number
  expectedSurfaceCenterMm: {
    x: number
    y: number
  }
  expectedMachineSpanXDeg: number | null
  expectedMachineSpanYDeg: number | null
  gcodeSpanXDeg: number | null
  gcodeSpanYDeg: number | null
  sourceRegionId: string
  gcodeMatchesMachineDegreeBbox: boolean
}

export type CalibrationPattern = {
  pattern: string
  ballDiameterMm: number
  coordinateModel: string
  previewAndGcodeShareSameProjectedPaths: boolean
  projectedVsGcodeMismatchSquareIds: string[]
  gcodeComparisonToleranceDeg: number
  squares: CalibrationSquare[]
}

export type XAxisCalibrationTick = {
  id: string
  label: string
  commandedXDeg: number
  emittedMachineXDeg: number
  machineDegreeBbox: CalibrationBbox | null
  gcodeBbox: CalibrationBbox | null
  expectedSurfaceArcFromPreviousMm: number | null
  gcodeMatchesMachineDegreeBbox: boolean
}

export type XAxisCalibrationPattern = {
  pattern: string
  ballDiameterMm: number
  ballCircumferenceMm: number
  expectedQuadrantArcMm: number
  previewAndGcodeShareSameProjectedPaths: boolean
  projectedVsGcodeMismatchTickIds: string[]
  gcodeComparisonToleranceDeg: number
  ticks: XAxisCalibrationTick[]
}

export type AnalyzeColor = {
  hex: string
  rgb: [number, number, number]
  pixel_count: number
  coverage: number
  luminance: number
}

export type ImageAnalysis = {
  width: number
  height: number
  colors: AnalyzeColor[]
}

export type JobSummary = {
  image_size: string
  selected_colors: string[]
  mask_pixel_count: number
  component_count: number
  toolpath_counts: Record<string, number>
  wall_path_count: number
  infill_path_count: number
  detail_trace_path_count: number
  travel_path_count: number
  gcode_line_count: number
  point_count: number
  estimated_runtime_seconds: number
  pen_lift_count: number
}

export type MachineState = {
  connected: boolean
  calibrated: boolean
  machine_position_trusted?: boolean
  emergency_stopped?: boolean
  running: boolean
  paused: boolean
  status: string
  progress_done: number
  progress_total: number
  run_started_at: number | null
  pause_started_at: number | null
  paused_duration_seconds: number
  current_gcode_line: number
  current_path_id: string | null
  current_path_kind?: string | null
  current_preview_point_index: number
  current_position_x?: number
  current_position_y?: number
  motor_hold_enabled?: boolean
  motors?: {
    method: string
    connected: boolean
    calibration_locked: boolean
    policy: string
    hold_active: boolean
    desired_dollar_1: number | null
    applied_dollar_1: number | null
    last_known_dollar_1: number | null
    x_expected_holding: boolean
    y_expected_holding: boolean
    applying: boolean
    queued_apply_reason: string | null
    last_apply_reason: string | null
    last_apply_ok: boolean | null
    last_error: string | null
  }
  stepper_hold_debug?: Record<string, unknown> | null
  last_summary: JobSummary | null
  last_job_finalization?: Record<string, unknown> | null
  last_timeout_debug?: Record<string, unknown> | null
  y_loop_test?: {
    enabled: boolean
    center_y: number
    distance: number
    feedrate: number
    dwell_sec: number
    phase: string
    cycles_completed: number
  }
  movement_test?: {
    active: boolean
    axis: string
    x_motor_holding: boolean
    y_motor_holding: boolean
    amplitude_deg: number
    feedrate: number
    cycle_count: number
  }
  streaming?: {
    mode: 'buffered' | 'sync'
    current_line: number
    current_path_id?: string | null
    current_path_kind?: string | null
    pending_buffer_chars: number
    pending_commands: number
    last_response_age_sec: number
    last_grbl_status: string | null
    ok_count: number
    error_count?: number
    sent_count: number
  }
  defaults: {
    pen_up_s: number
    pen_down_s: number
    pen_up_dwell_ms: number
    pen_down_dwell_ms: number
    servo_ramp_enabled: boolean
    servo_ramp_step: number
    servo_ramp_delay_ms: number
  }
}

export type GenerateResponse = {
  ok: true
  gcode: string[]
  preview: PreviewPath[]
  mask_preview: string | null
  selected_colors: string[]
  summary: JobSummary
  stage_counts: Record<string, unknown>
  effective_settings: {
    line_thickness_mm: number
    infill_spacing_mm: number
    custom_infill_spacing: boolean
    wall_count: number
    fill_density: number
  }
  calibrationPattern?: CalibrationPattern | null
  xAxisCalibrationPattern?: XAxisCalibrationPattern | null
}

export type ApiSuccess<T> = T & {
  ok: true
  command?: string
  response?: string
}

export type ApiError = {
  ok: false
  error: string
}

export type AppDefaults = {
  xMaxFeed: number
  yMaxFeed: number
  xAcceleration: number
  yAcceleration: number
  drawFeed: number
  travelFeed: number
  lineThicknessMm: number
  penUpS: number
  penDownS: number
  penUpDwellMs: number
  penDownDwellMs: number
  servoRampEnabled: boolean
  servoRampStep: number
  servoRampDelayMs: number
  sampleStepDeg: number
  marginPercent: number
  rotationDeg: number
  wallCount: number
  infillDensity: number
  infillSpacingMm: number
  customInfillSpacingEnabled?: boolean
  infillAngleDeg: number
  fillStrategy?: 'horizontal_scanline' | 'rotated_scanline' | 'adaptive_angle' | 'crosshatch'
  alternateFillAngleDeg?: number
  minFillAreaMm2: number
  minFillWidthMm: number
  simplifyToleranceMm: number
  removeDuplicatePaths: boolean
  minSegmentLengthMm: number
  allowPenDownInfillConnectors: boolean
  thinDetailMode: boolean
  thinDetailMinAreaMm2: number
  thinDetailSimplifyMm: number
  thinDetailOverlap: boolean
  rasterMaxColors: number
  rasterColorTolerance: number
  rasterMinComponentAreaPx: number
  rasterMaskOpenRadiusPx: number
  rasterMaskCloseRadiusPx: number
  rasterMinRegionAreaPx: number
  rasterRegionSimplifyPx: number
  outlineAfterFill: boolean
  streamingMode?: 'buffered' | 'sync'
  yLoopDistance?: number
  yLoopFeedrate?: number
  yLoopDwellSec?: number
}

export type AppConfig = {
  defaults: AppDefaults
}
