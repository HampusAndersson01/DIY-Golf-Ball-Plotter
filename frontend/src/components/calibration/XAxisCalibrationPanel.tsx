import { useMemo } from 'react'

import { useAppStore } from '../../store/appStore'
import { analyzeXAxisCalibrationPattern } from './xAxisCalibrationMath'

type Props = {
  generating: boolean
  onGenerate: () => void
}

export function XAxisCalibrationPanel({ generating, onGenerate }: Props) {
  const pattern = useAppStore((state) => state.xAxisCalibrationPattern)
  const measurements = useAppStore((state) => state.xAxisCalibrationMeasurements)
  const updateMeasurement = useAppStore((state) => state.updateXAxisCalibrationMeasurement)
  const analysis = useMemo(() => analyzeXAxisCalibrationPattern(pattern, measurements), [measurements, pattern])

  return (
    <section className="panel">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Calibration</div>
          <h2>X Rotary Test</h2>
        </div>
      </div>

      <p className="panel-note">
        This test draws short vertical ticks at commanded X positions 0, 90, 180, 270, and 360 degrees. The 0 and 360 ticks should overlap.
      </p>
      <p className="panel-copy muted">
        Measure the arc distance along the ball surface between adjacent ticks, not the straight-line camera distance.
      </p>

      <div className="stack-row" style={{ marginTop: '10px' }}>
        <button className="button primary" disabled={generating} onClick={onGenerate} type="button">
          {generating ? 'Generating X rotary test...' : 'Generate X rotary calibration G-code'}
        </button>
      </div>

      {!pattern || !analysis ? (
        <p className="panel-copy muted" style={{ marginTop: '10px' }}>
          Generate the rotary test to verify pure X rotation independently from normal artwork projection.
        </p>
      ) : (
        <>
          <div className="summary-grid" style={{ marginTop: '10px' }}>
            <div><span>Pattern</span><strong>{pattern.pattern}</strong></div>
            <div><span>Ball diameter</span><strong>{pattern.ballDiameterMm.toFixed(2)} mm</strong></div>
            <div><span>Circumference</span><strong>{pattern.ballCircumferenceMm.toFixed(3)} mm</strong></div>
            <div><span>Expected 90 deg arc</span><strong>{pattern.expectedQuadrantArcMm.toFixed(3)} mm</strong></div>
          </div>

          <div className="summary-grid" style={{ marginTop: '10px' }}>
            {pattern.ticks.map((tick) => (
              <div key={tick.id}>
                <span>{tick.label}</span>
                <strong>{tick.commandedXDeg.toFixed(0)} deg cmd / {tick.emittedMachineXDeg.toFixed(0)} deg emit</strong>
              </div>
            ))}
          </div>

          <div className="calibration-table-wrap">
            <table className="calibration-table">
              <thead>
                <tr>
                  <th>Segment</th>
                  <th>Expected arc</th>
                  <th>Actual arc</th>
                  <th>Error</th>
                  <th>Ratio</th>
                </tr>
              </thead>
              <tbody>
                {analysis.segments.map((segment) => (
                  <tr key={segment.id}>
                    <td><strong>{segment.label}</strong></td>
                    <td>{segment.expectedArcMm.toFixed(3)} mm</td>
                    <td>
                      <input
                        onChange={(event) => updateMeasurement(segment.id, event.target.value)}
                        step="0.01"
                        type="number"
                        value={measurements[segment.id]?.actualArcMm ?? ''}
                      />
                    </td>
                    <td>{formatDelta(segment.errorMm)}</td>
                    <td>{formatPercent(segment.percent)}</td>
                  </tr>
                ))}
                <tr>
                  <td><strong>0 deg -&gt; 360 deg overlap error</strong></td>
                  <td>0.000 mm</td>
                  <td>
                    <input
                      onChange={(event) => updateMeasurement('overlap_error', event.target.value)}
                      step="0.01"
                      type="number"
                      value={measurements.overlap_error?.actualArcMm ?? ''}
                    />
                  </td>
                  <td>{formatDelta(analysis.overlapErrorMm)}</td>
                  <td>--</td>
                </tr>
              </tbody>
            </table>
          </div>

          <div className="summary-grid" style={{ marginTop: '10px' }}>
            <div><span>Average X ratio</span><strong>{formatRatio(analysis.averageRatio)}</strong></div>
            <div><span>Suggested X correction</span><strong>{formatFactor(analysis.suggestedCorrectionFactor)}</strong></div>
            <div><span>0/360 overlap error</span><strong>{analysis.overlapErrorMm === null ? '--' : `${analysis.overlapErrorMm.toFixed(3)} mm`}</strong></div>
            <div><span>Preview/G-code</span><strong>{pattern.previewAndGcodeShareSameProjectedPaths ? 'Matched' : 'Mismatch'}</strong></div>
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

function formatPercent(value: number | null) {
  if (value === null || !Number.isFinite(value)) return '--'
  return `${value.toFixed(1)}%`
}

function formatRatio(value: number | null) {
  if (value === null || !Number.isFinite(value)) return '--'
  return `${(value * 100).toFixed(1)}%`
}

function formatFactor(value: number | null) {
  if (value === null || !Number.isFinite(value)) return '--'
  return `x${value.toFixed(3)}`
}

function formatDelta(value: number | null) {
  if (value === null || !Number.isFinite(value)) return '--'
  return `${value >= 0 ? '+' : ''}${value.toFixed(3)} mm`
}
