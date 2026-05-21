import { useMemo } from 'react'

import type { CalibrationSquare } from '../../api/types'
import { useAppStore } from '../../store/appStore'
import { analyzeCalibrationPattern } from './calibrationMath'

type Props = {
  generating: boolean
  onGenerate: () => void
}

export function CalibrationPatternPanel({ generating, onGenerate }: Props) {
  const calibrationPattern = useAppStore((state) => state.calibrationPattern)
  const calibrationMeasurements = useAppStore((state) => state.calibrationMeasurements)
  const updateCalibrationMeasurement = useAppStore((state) => state.updateCalibrationMeasurement)
  const setCalibrationSkipped = useAppStore((state) => state.setCalibrationSkipped)

  const analysis = useMemo(
    () => analyzeCalibrationPattern(calibrationPattern, calibrationMeasurements),
    [calibrationMeasurements, calibrationPattern],
  )

  return (
    <section className="panel">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Calibration</div>
          <h2>3x3 Square Test</h2>
        </div>
      </div>

      <p className="panel-note">
        Generate the filled 3x3 square matrix from canonical surface-mm geometry, then compare the physical print against the expected surface and machine spans.
      </p>
      <p className="panel-copy muted">
        Measure the printed square directly on the ball surface as well as possible. Do not measure from a photo unless the photo is corrected for perspective.
      </p>

      <div className="stack-row" style={{ marginTop: '10px' }}>
        <button className="button primary" disabled={generating} onClick={onGenerate} type="button">
          {generating ? 'Generating calibration pattern...' : 'Generate 3x3 calibration G-code'}
        </button>
      </div>

      {!calibrationPattern || !analysis ? (
        <p className="panel-copy muted" style={{ marginTop: '10px' }}>
          Generate the diagnostic pattern to inspect expected square sizes, machine-degree spans, parsed G-code spans, and post-print measurement ratios.
        </p>
      ) : (
        <>
          <div className="summary-grid" style={{ marginTop: '10px' }}>
            <div><span>Pattern</span><strong>{calibrationPattern.pattern}</strong></div>
            <div><span>Ball diameter</span><strong>{formatMm(calibrationPattern.ballDiameterMm)}</strong></div>
            <div><span>Projected/G-code</span><strong>{calibrationPattern.previewAndGcodeShareSameProjectedPaths ? 'Matched' : 'Mismatch'}</strong></div>
            <div><span>Squares</span><strong>{calibrationPattern.squares.length}</strong></div>
          </div>

          <div className="calibration-table-wrap">
            <table className="calibration-table">
              <thead>
                <tr>
                  <th>Square</th>
                  <th>Expected surface size</th>
                  <th>Expected center</th>
                  <th>Expected machine span</th>
                  <th>Parsed G-code span</th>
                  <th>Actual width</th>
                  <th>Actual height</th>
                  <th>Width error</th>
                  <th>Height error</th>
                  <th>Width ratio</th>
                  <th>Height ratio</th>
                  <th>Skip</th>
                </tr>
              </thead>
              <tbody>
                {calibrationPattern.squares.map((square) => {
                  const squareAnalysis = analysis.squares.find((entry) => entry.squareId === square.id)
                  if (!squareAnalysis) return null
                  const measurement = calibrationMeasurements[square.id] ?? { actualWidthMm: '', actualHeightMm: '', skipped: false }
                  return (
                    <tr key={square.id} className={squareAnalysis.skipped ? 'is-skipped' : ''}>
                      <td>
                        <strong>{square.label}</strong>
                      </td>
                      <td>{formatMm(square.expectedSurfaceWidthMm)} x {formatMm(square.expectedSurfaceHeightMm)}</td>
                      <td>{formatCenter(square)}</td>
                      <td>{formatSpan(square.expectedMachineSpanXDeg)} x {formatSpan(square.expectedMachineSpanYDeg)}</td>
                      <td>{formatSpan(square.gcodeSpanXDeg)} x {formatSpan(square.gcodeSpanYDeg)}</td>
                      <td>
                        <input
                          disabled={squareAnalysis.skipped}
                          onChange={(event) => updateCalibrationMeasurement(square.id, 'actualWidthMm', event.target.value)}
                          step="0.01"
                          type="number"
                          value={measurement.actualWidthMm}
                        />
                      </td>
                      <td>
                        <input
                          disabled={squareAnalysis.skipped}
                          onChange={(event) => updateCalibrationMeasurement(square.id, 'actualHeightMm', event.target.value)}
                          step="0.01"
                          type="number"
                          value={measurement.actualHeightMm}
                        />
                      </td>
                      <td>{formatDeltaMm(squareAnalysis.widthErrorMm)}</td>
                      <td>{formatDeltaMm(squareAnalysis.heightErrorMm)}</td>
                      <td>{formatPercent(squareAnalysis.widthPercent)}</td>
                      <td>{formatPercent(squareAnalysis.heightPercent)}</td>
                      <td>
                        <label className="toggle">
                          <input
                            checked={squareAnalysis.skipped}
                            onChange={(event) => setCalibrationSkipped(square.id, event.target.checked)}
                            type="checkbox"
                          />
                          <span>Invalid</span>
                        </label>
                      </td>
                    </tr>
                  )
                })}
              </tbody>
            </table>
          </div>

          <div className="summary-grid" style={{ marginTop: '10px' }}>
            <div><span>Average width ratio</span><strong>{formatPercentFromRatio(analysis.widthRatio.average)}</strong></div>
            <div><span>Average height ratio</span><strong>{formatPercentFromRatio(analysis.heightRatio.average)}</strong></div>
            <div><span>Min/max width ratio</span><strong>{formatRange(analysis.widthRatio.min, analysis.widthRatio.max)}</strong></div>
            <div><span>Min/max height ratio</span><strong>{formatRange(analysis.heightRatio.min, analysis.heightRatio.max)}</strong></div>
            <div><span>Average width error</span><strong>{formatDeltaMm(analysis.widthError.average)}</strong></div>
            <div><span>Average height error</span><strong>{formatDeltaMm(analysis.heightError.average)}</strong></div>
          </div>

          <div className="details-panel" style={{ marginTop: '10px' }}>
            <div className="panel-kicker">Diagnosis</div>
            {analysis.diagnosis.map((entry) => (
              <p key={entry} className="panel-copy" style={{ marginTop: '6px' }}>{entry}</p>
            ))}
          </div>
        </>
      )}
    </section>
  )
}

function formatMm(value: number | null) {
  if (value === null || !Number.isFinite(value)) return '--'
  return `${value.toFixed(3)} mm`
}

function formatSpan(value: number | null) {
  if (value === null || !Number.isFinite(value)) return '--'
  return `${value.toFixed(4)} deg`
}

function formatPercent(value: number | null) {
  if (value === null || !Number.isFinite(value)) return '--'
  return `${value.toFixed(1)}%`
}

function formatPercentFromRatio(value: number | null) {
  if (value === null || !Number.isFinite(value)) return '--'
  return `${(value * 100).toFixed(1)}%`
}

function formatDeltaMm(value: number | null) {
  if (value === null || !Number.isFinite(value)) return '--'
  return `${value >= 0 ? '+' : ''}${value.toFixed(3)} mm`
}

function formatRange(min: number | null, max: number | null) {
  if (min === null || max === null || !Number.isFinite(min) || !Number.isFinite(max)) return '--'
  return `${(min * 100).toFixed(1)}% to ${(max * 100).toFixed(1)}%`
}

function formatCenter(square: CalibrationSquare) {
  return `${square.expectedSurfaceCenterMm.x.toFixed(3)} mm, ${square.expectedSurfaceCenterMm.y.toFixed(3)} mm`
}
