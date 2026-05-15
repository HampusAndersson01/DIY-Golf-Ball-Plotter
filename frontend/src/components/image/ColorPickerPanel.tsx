import type { ImageAnalysis } from '../../api/types'

type Props = {
  analysis: ImageAnalysis | null
  selectedColors: string[]
  onToggle: (hex: string) => void
}

export function ColorPickerPanel({ analysis, selectedColors, onToggle }: Props) {
  return (
    <section className="panel">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Mask</div>
          <h2>Printable Colors</h2>
        </div>
        <span className="badge muted">{selectedColors.length} selected</span>
      </div>

      <div className="swatch-grid">
        {analysis?.colors?.length ? (
          analysis.colors.map((color) => {
            const active = selectedColors.includes(color.hex)
            return (
              <button
                key={color.hex}
                className={`swatch-card ${active ? 'is-active' : ''}`}
                onClick={() => onToggle(color.hex)}
                type="button"
              >
                <span className="swatch-chip" style={{ background: color.hex }} />
                <strong>{color.hex}</strong>
                <small>{(color.coverage * 100).toFixed(1)}% coverage</small>
              </button>
            )
          })
        ) : (
          <div className="panel-copy muted">Analyze the image to populate selectable color swatches.</div>
        )}
      </div>
    </section>
  )
}
