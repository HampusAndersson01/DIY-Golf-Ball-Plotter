import type { ChangeEvent } from 'react'
import { useMemo } from 'react'

import type { AnalyzeColor, ImageAnalysis } from '../../api/types'
import type { OriginAnchor } from '../../store/appStore'
import { useAppStore } from '../../store/appStore'
import { parseLocaleNumber } from '../../utils/numbers'

type Props = {
  analysis: ImageAnalysis | null
  selectedColors: string[]
  imagePreviewUrl: string | null
  canGenerate: boolean
  onToggleColor: (colorId: string) => void
  onFileChange: (event: ChangeEvent<HTMLInputElement>) => void
  onGenerate: () => void
}

const ORIGIN_ANCHOR_OPTIONS: Array<{ value: OriginAnchor; label: string }> = [
  { value: 'center', label: 'Center' },
  { value: 'min-x', label: 'Left edge / Min X' },
  { value: 'max-x', label: 'Right edge / Max X' },
  { value: 'min-y', label: 'Bottom edge / Min Y' },
  { value: 'max-y', label: 'Top edge / Max Y' },
  { value: 'top-left', label: 'Top left' },
  { value: 'top-center', label: 'Top center' },
  { value: 'top-right', label: 'Top right' },
  { value: 'center-left', label: 'Center left' },
  { value: 'center-right', label: 'Center right' },
  { value: 'bottom-left', label: 'Bottom left' },
  { value: 'bottom-center', label: 'Bottom center' },
  { value: 'bottom-right', label: 'Bottom right' },
  { value: 'custom', label: 'Custom' },
]

export function PrintSetupPanel({
  analysis,
  selectedColors,
  imagePreviewUrl,
  canGenerate,
  onToggleColor,
  onFileChange,
  onGenerate,
}: Props) {
  const settings = useAppStore((state) => state.settings)!
  const imageFile = useAppStore((state) => state.imageFile)
  const gcode = useAppStore((state) => state.gcode)
  const summary = useAppStore((state) => state.summary)
  const updateSetting = useAppStore((state) => state.updateSetting)
  const colors = useMemo(() => analysis?.colors ?? [], [analysis])
  const selectedSet = useMemo(() => new Set(selectedColors), [selectedColors])
  const selectedColorRows = useMemo(
    () => selectedColors
      .map((id) => colors.find((color) => color.id === id))
      .filter((color): color is AnalyzeColor => Boolean(color)),
    [colors, selectedColors],
  )
  const firstSelectedColor = selectedColorRows[0] ?? null
  const readyToGenerate = Boolean(imageFile && selectedColors.length)
  const effectiveInfillSpacingMm = settings.customInfillSpacingEnabled ? settings.infillSpacingMm : settings.lineThicknessMm
  const runtimeText = summary ? formatDuration(summary.estimated_runtime_seconds) : '--'
  const lineCount = summary?.gcode_line_count ?? gcode.length

  return (
    <section className="sidebar-panel sidebar-panel--surface print-setup-panel">
      <header className="print-setup-head">
        <div>
          <h2>Print Setup</h2>
          <p className="panel-note">Prepare image, position, pen width, and generate G-code.</p>
        </div>
      </header>

      <section className="print-setup-section">
        <div className="print-setup-section__head">
          <h3>Image</h3>
          <span className={`badge ${analysis ? 'good' : 'muted'}`}>{analysis ? 'Analyzed' : 'Awaiting image'}</span>
        </div>

        <label className="upload-zone print-setup-upload-zone">
          <input accept=".png,.jpg,.jpeg,image/png,image/jpeg" onChange={onFileChange} type="file" />
          <span>{imagePreviewUrl ? 'Replace Image' : 'Import Image'}</span>
        </label>

        <div className="thumb-frame print-setup-thumb">
          {imagePreviewUrl ? <img alt="Selected image" src={imagePreviewUrl} /> : <span>Image preview</span>}
        </div>

        <p className="panel-note">Colors are analyzed automatically.</p>
      </section>

      <section className="print-setup-section">
        <div className="print-setup-section__head">
          <h3>Colors</h3>
          <div className="print-setup-meta-row">
            <span className="badge muted">{selectedColors.length} selected</span>
            <span className="badge muted">{colors.length} detected</span>
          </div>
        </div>

        <div className="print-setup-color-row" role="status" aria-live="polite">
          <span className="print-setup-color-row__label">Printable Colors</span>
          {firstSelectedColor ? (
            <button
              className="print-setup-selected-chip"
              onClick={() => onToggleColor(firstSelectedColor.id)}
              type="button"
            >
              <i style={{ backgroundColor: firstSelectedColor.hex }} />
              <strong>{firstSelectedColor.hex}</strong>
              <small>{firstSelectedColor.coverage_percent.toFixed(1)}%</small>
            </button>
          ) : (
            <span className="muted">Select at least one color.</span>
          )}
        </div>

        {colors.length ? (
          <div className="print-setup-swatch-grid" aria-label="Detected color swatches">
            {colors.map((color) => (
              <button
                key={color.id}
                className={`print-setup-swatch ${selectedSet.has(color.id) ? 'is-selected' : ''}`}
                onClick={() => onToggleColor(color.id)}
                style={{ backgroundColor: color.hex }}
                title={`${color.hex} ${color.coverage_percent.toFixed(1)}%`}
                type="button"
              />
            ))}
          </div>
        ) : (
          <p className="panel-note">Import an image to populate printable colors.</p>
        )}
      </section>

      <section className="print-setup-section">
        <div className="print-setup-section__head">
          <h3>Position</h3>
        </div>

        <div className="print-setup-grid">
          <label className="inline-number-field" htmlFor="artworkScalePercent">
            <span>Scale</span>
            <div className="inline-number-field__control">
              <input
                id="artworkScalePercent"
                max="200"
                min="10"
                onBlur={(event) => {
                  const parsed = parseLocaleNumber(event.target.value)
                  const nextValue = Number.isFinite(parsed) ? Math.min(200, Math.max(10, Math.round(parsed))) : 100
                  updateSetting('artworkScalePercent', nextValue)
                }}
                onChange={(event) => updateSetting('artworkScalePercent', parseLocaleNumber(event.target.value))}
                step="1"
                type="number"
                value={settings.artworkScalePercent}
              />
              <small>%</small>
            </div>
          </label>

          <label className="inline-number-field" htmlFor="originAnchor">
            <span>Origin anchor</span>
            <select
              id="originAnchor"
              onChange={(event) => updateSetting('originAnchor', event.target.value as OriginAnchor)}
              value={settings.originAnchor}
            >
              {ORIGIN_ANCHOR_OPTIONS.map((option) => (
                <option key={option.value} value={option.value}>
                  {option.label}
                </option>
              ))}
            </select>
          </label>

          <label className="inline-number-field" htmlFor="originOffsetXmm">
            <span>Manual offset X</span>
            <div className="inline-number-field__control">
              <input
                id="originOffsetXmm"
                onChange={(event) => updateSetting('originOffsetXmm', parseLocaleNumber(event.target.value))}
                step="0.01"
                type="number"
                value={settings.originOffsetXmm}
              />
              <small>mm</small>
            </div>
          </label>

          <label className="inline-number-field" htmlFor="originOffsetYmm">
            <span>Manual offset Y</span>
            <div className="inline-number-field__control">
              <input
                id="originOffsetYmm"
                onChange={(event) => updateSetting('originOffsetYmm', parseLocaleNumber(event.target.value))}
                step="0.01"
                type="number"
                value={settings.originOffsetYmm}
              />
              <small>mm</small>
            </div>
          </label>
        </div>
      </section>

      <section className="print-setup-section">
        <div className="print-setup-section__head">
          <h3>Pen</h3>
        </div>

        <div className="print-setup-grid print-setup-grid--single">
          <label className="inline-number-field" htmlFor="lineThicknessMm">
            <span>Pen width</span>
            <div className="inline-number-field__control">
              <input
                id="lineThicknessMm"
                onChange={(event) => updateSetting('lineThicknessMm', parseLocaleNumber(event.target.value))}
                step="0.01"
                type="number"
                value={settings.lineThicknessMm}
              />
              <small>mm</small>
            </div>
          </label>
        </div>

        <p className="panel-note">Infill spacing: {settings.customInfillSpacingEnabled ? `${effectiveInfillSpacingMm.toFixed(2)} mm` : `Auto ${effectiveInfillSpacingMm.toFixed(2)} mm`}.</p>
      </section>

      <section className="print-setup-section print-setup-section--generate">
        <button className="button primary print-setup-generate" disabled={!canGenerate} onClick={onGenerate} type="button">
          Generate G-code
        </button>

        <p className={`print-setup-readiness ${readyToGenerate ? 'is-ready' : 'is-blocked'}`}>
          {readyToGenerate ? 'Ready to generate' : 'Import an image and select at least one color.'}
        </p>

        {(summary || gcode.length > 0) ? (
          <div className="print-setup-generated-summary">
            <strong>Generated</strong>
            <div>
              <span>Runtime</span>
              <em>{runtimeText}</em>
            </div>
            <div>
              <span>Pen lifts</span>
              <em>{summary?.pen_lift_count ?? '--'}</em>
            </div>
            <div>
              <span>Lines</span>
              <em>{lineCount.toLocaleString()}</em>
            </div>
          </div>
        ) : null}
      </section>
    </section>
  )
}

function formatDuration(totalSeconds: number) {
  const wholeSeconds = Math.max(0, Math.round(totalSeconds))
  const minutes = Math.floor(wholeSeconds / 60)
  const seconds = wholeSeconds % 60
  return `${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`
}
