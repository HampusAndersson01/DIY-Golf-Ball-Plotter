import { useMemo, useState } from 'react'

import type { ImageAnalysis } from '../../api/types'

type Props = {
  analysis: ImageAnalysis | null
  selectedColors: string[]
  onToggle: (colorId: string) => void
}

export function ColorPickerPanel({ analysis, selectedColors, onToggle }: Props) {
  const [expanded, setExpanded] = useState(false)
  const colors = analysis?.colors ?? []
  const selectedSet = useMemo(() => new Set(selectedColors), [selectedColors])
  const selected = colors.filter((color) => selectedSet.has(color.id))
  const remaining = colors.filter((color) => !selectedSet.has(color.id))
  const compactRemaining = expanded ? remaining : remaining.slice(0, 12)
  const hasAnalysis = analysis !== null

  return (
    <section className="panel">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Mask</div>
          <h2>Printable Colors</h2>
        </div>
        <span className="badge muted">{selectedColors.length} selected</span>
      </div>

      {!colors.length ? (
        <div className="panel-copy muted">
          {hasAnalysis ? 'No printable colors detected. Transparent or nearly transparent pixels were ignored.' : 'Analyze the image to populate selectable color swatches.'}
        </div>
      ) : null}

      {selected.length ? (
        <div className="compact-color-section">
          <div className="section-label">Selected</div>
          <div className="selected-swatch-list">
            {selected.map((color) => (
              <button
                key={color.id}
                className="selected-swatch"
                onClick={() => onToggle(color.id)}
                type="button"
              >
                <i style={{ background: color.hex }} />
                <strong>{color.hex}</strong>
                <small>{color.coverage_percent.toFixed(1)}%</small>
              </button>
            ))}
          </div>
        </div>
      ) : null}

      {remaining.length ? (
        <div className="compact-color-section">
          <div className="stack-row">
            <div className="section-label">Available</div>
            {remaining.length > 12 ? (
              <button className="text-button" onClick={() => setExpanded((value) => !value)} type="button">
                {expanded ? 'Show fewer' : `Show all (${remaining.length})`}
              </button>
            ) : null}
          </div>
          <div className="swatch-dot-grid">
            {compactRemaining.map((color) => (
              <button
                key={color.id}
                className="swatch-dot"
                onClick={() => onToggle(color.id)}
                style={{ background: color.hex }}
                title={`${color.hex} ${color.coverage_percent.toFixed(1)}%`}
                type="button"
              />
            ))}
          </div>
        </div>
      ) : null}
    </section>
  )
}
