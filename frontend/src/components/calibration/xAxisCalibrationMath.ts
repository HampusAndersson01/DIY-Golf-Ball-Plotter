import type { XAxisCalibrationPattern } from '../../api/types'
import type { XAxisCalibrationMeasurement } from '../../store/appStore'

type SegmentMetric = {
  id: string
  label: string
  expectedArcMm: number
  actualArcMm: number | null
  ratio: number | null
  percent: number | null
  errorMm: number | null
}

export type XAxisCalibrationAnalysis = {
  segments: SegmentMetric[]
  overlapErrorMm: number | null
  averageRatio: number | null
  suggestedCorrectionFactor: number | null
  diagnosis: string[]
}

function parseMeasurement(value: string | undefined): number | null {
  if (!value) return null
  const normalized = Number(value.replace(',', '.'))
  return Number.isFinite(normalized) ? normalized : null
}

export function analyzeXAxisCalibrationPattern(
  pattern: XAxisCalibrationPattern | null,
  measurements: Record<string, XAxisCalibrationMeasurement>,
): XAxisCalibrationAnalysis | null {
  if (!pattern) return null

  const segments: SegmentMetric[] = []
  for (let index = 1; index < pattern.ticks.length; index += 1) {
    const previousTick: XAxisCalibrationPattern['ticks'][number] = pattern.ticks[index - 1]
    const currentTick: XAxisCalibrationPattern['ticks'][number] = pattern.ticks[index]
    const measurementId = `${previousTick.id}_to_${currentTick.id}`
    const actualArcMm = parseMeasurement(measurements[measurementId]?.actualArcMm)
    const ratio = actualArcMm === null ? null : actualArcMm / pattern.expectedQuadrantArcMm
    segments.push({
      id: measurementId,
      label: `${previousTick.label} -> ${currentTick.label}`,
      expectedArcMm: pattern.expectedQuadrantArcMm,
      actualArcMm,
      ratio,
      percent: ratio === null ? null : ratio * 100,
      errorMm: actualArcMm === null ? null : actualArcMm - pattern.expectedQuadrantArcMm,
    })
  }

  const validRatios = segments.map((segment) => segment.ratio).filter((value): value is number => value !== null)
  const averageRatio = validRatios.length ? validRatios.reduce((sum, value) => sum + value, 0) / validRatios.length : null
  const suggestedCorrectionFactor = averageRatio && averageRatio > 0 ? 1.0 / averageRatio : null
  const overlapErrorMm = parseMeasurement(measurements.overlap_error?.actualArcMm)

  const diagnosis: string[] = []
  if (averageRatio !== null) {
    if (averageRatio < 0.95) {
      diagnosis.push('The measured arc spacing is consistently smaller than expected. That points to X rotary under-travel, slip, or X steps/ratio calibration error.')
    } else if (averageRatio > 1.05) {
      diagnosis.push('The measured arc spacing is consistently larger than expected. That points to X rotary over-travel or an X calibration ratio that is too large.')
    }
    const minRatio = Math.min(...validRatios)
    const maxRatio = Math.max(...validRatios)
    if ((maxRatio - minRatio) >= 0.05) {
      diagnosis.push('The quadrant spacings are not uniform. That points more toward slip, backlash, or mechanical eccentricity than a single global X scale constant.')
    }
  }
  if (overlapErrorMm !== null && overlapErrorMm > 0.5) {
    diagnosis.push('The 0 deg and 360 deg ticks do not overlap closely. That points to cumulative slip or backlash over one revolution.')
  }
  if (pattern.projectedVsGcodeMismatchTickIds.length > 0) {
    diagnosis.push('Parsed G-code does not match the emitted machine-degree tick geometry for one or more ticks. That would indicate a software preview/G-code mismatch.')
  }
  if (!diagnosis.length) {
    diagnosis.push('Enter the measured arc distances between adjacent ticks and the 0/360 overlap error to estimate an X correction factor.')
  }

  return {
    segments,
    overlapErrorMm,
    averageRatio,
    suggestedCorrectionFactor,
    diagnosis,
  }
}
