import type { CalibrationPattern } from '../../api/types'
import type { CalibrationMeasurement } from '../../store/appStore'

type SquareMetric = {
  squareId: string
  label: string
  skipped: boolean
  actualWidthMm: number | null
  actualHeightMm: number | null
  widthErrorMm: number | null
  heightErrorMm: number | null
  widthRatio: number | null
  heightRatio: number | null
  widthPercent: number | null
  heightPercent: number | null
}

type RatioRange = {
  average: number | null
  min: number | null
  max: number | null
}

type ErrorAverage = {
  average: number | null
}

export type CalibrationAnalysis = {
  squares: SquareMetric[]
  widthRatio: RatioRange
  heightRatio: RatioRange
  widthError: ErrorAverage
  heightError: ErrorAverage
  diagnosis: string[]
}

function parseMeasurement(value: string | undefined): number | null {
  if (!value) return null
  const normalized = Number(value.replace(',', '.'))
  return Number.isFinite(normalized) ? normalized : null
}

function summarizeRatios(values: Array<number | null>): RatioRange {
  const valid = values.filter((value): value is number => typeof value === 'number' && Number.isFinite(value))
  if (!valid.length) {
    return { average: null, min: null, max: null }
  }
  return {
    average: valid.reduce((sum, value) => sum + value, 0) / valid.length,
    min: Math.min(...valid),
    max: Math.max(...valid),
  }
}

function summarizeErrors(values: Array<number | null>): ErrorAverage {
  const valid = values.filter((value): value is number => typeof value === 'number' && Number.isFinite(value))
  if (!valid.length) {
    return { average: null }
  }
  return {
    average: valid.reduce((sum, value) => sum + value, 0) / valid.length,
  }
}

function groupSpreadBy<T extends 'row' | 'col'>(
  pattern: CalibrationPattern,
  squares: SquareMetric[],
  key: T,
  metric: 'widthRatio' | 'heightRatio',
): number {
  const grouped = new Map<number, number[]>()
  for (const square of squares) {
    if (square.skipped) continue
    const source = pattern.squares.find((entry) => entry.id === square.squareId)
    const value = square[metric]
    if (!source || value === null) continue
    const bucket = grouped.get(source[key]) ?? []
    bucket.push(value)
    grouped.set(source[key], bucket)
  }
  const averages = [...grouped.values()]
    .filter((values) => values.length)
    .map((values) => values.reduce((sum, value) => sum + value, 0) / values.length)
  if (averages.length < 2) return 0
  return Math.max(...averages) - Math.min(...averages)
}

export function analyzeCalibrationPattern(
  pattern: CalibrationPattern | null,
  measurements: Record<string, CalibrationMeasurement>,
): CalibrationAnalysis | null {
  if (!pattern) return null

  const squares = pattern.squares.map((square) => {
    const measurement = measurements[square.id] ?? { actualWidthMm: '', actualHeightMm: '', skipped: false }
    const actualWidthMm = parseMeasurement(measurement.actualWidthMm)
    const actualHeightMm = parseMeasurement(measurement.actualHeightMm)
    const widthRatio = actualWidthMm === null ? null : actualWidthMm / square.expectedSurfaceWidthMm
    const heightRatio = actualHeightMm === null ? null : actualHeightMm / square.expectedSurfaceHeightMm
    return {
      squareId: square.id,
      label: square.label,
      skipped: measurement.skipped,
      actualWidthMm,
      actualHeightMm,
      widthErrorMm: actualWidthMm === null ? null : actualWidthMm - square.expectedSurfaceWidthMm,
      heightErrorMm: actualHeightMm === null ? null : actualHeightMm - square.expectedSurfaceHeightMm,
      widthRatio,
      heightRatio,
      widthPercent: widthRatio === null ? null : widthRatio * 100,
      heightPercent: heightRatio === null ? null : heightRatio * 100,
    }
  })

  const activeSquares = squares.filter((square) => !square.skipped)
  const widthRatio = summarizeRatios(activeSquares.map((square) => square.widthRatio))
  const heightRatio = summarizeRatios(activeSquares.map((square) => square.heightRatio))
  const widthError = summarizeErrors(activeSquares.map((square) => square.widthErrorMm))
  const heightError = summarizeErrors(activeSquares.map((square) => square.heightErrorMm))

  const diagnosis: string[] = []
  const averageWidthRatio = widthRatio.average
  const averageHeightRatio = heightRatio.average
  const rowWidthSpread = groupSpreadBy(pattern, activeSquares, 'row', 'widthRatio')
  const rowHeightSpread = groupSpreadBy(pattern, activeSquares, 'row', 'heightRatio')
  const colWidthSpread = groupSpreadBy(pattern, activeSquares, 'col', 'widthRatio')
  const colHeightSpread = groupSpreadBy(pattern, activeSquares, 'col', 'heightRatio')

  if (averageWidthRatio !== null && averageHeightRatio !== null) {
    if (averageWidthRatio < 0.95 && Math.abs(averageHeightRatio - 1.0) <= 0.05) {
      diagnosis.push('Widths are consistently undersized while heights stay near target. Likely X scale, ball diameter, X calibration, or pen-offset/contact issue.')
    }
    if (averageHeightRatio < 0.95 && Math.abs(averageWidthRatio - 1.0) <= 0.05) {
      diagnosis.push('Heights are consistently undersized while widths stay near target. Likely Y scale or arm calibration issue.')
    }
    if (Math.max(rowWidthSpread, rowHeightSpread, colWidthSpread, colHeightSpread) >= 0.05) {
      diagnosis.push('Ratios vary materially by row or column. Likely kinematic model, pen-center alignment, or contact geometry issue across the ball surface.')
    }
    if (pattern.projectedVsGcodeMismatchSquareIds.length > 0) {
      diagnosis.push('Projected machine bbox and parsed G-code bbox differ for one or more squares. That points to a preview/G-code geometry mismatch in software.')
    } else if (
      (averageWidthRatio < 0.98 || averageWidthRatio > 1.02 || averageHeightRatio < 0.98 || averageHeightRatio > 1.02)
      && activeSquares.some((square) => square.widthRatio !== null || square.heightRatio !== null)
    ) {
      diagnosis.push('Projected spans and parsed G-code agree, but the physical print differs. That points to mechanical calibration, pen alignment, or contact behavior.')
    }
  }

  if (!diagnosis.length) {
    diagnosis.push('Enter measured values to compare the physical print against the canonical surface-mm and projected machine spans.')
  }

  return {
    squares,
    widthRatio,
    heightRatio,
    widthError,
    heightError,
    diagnosis,
  }
}
