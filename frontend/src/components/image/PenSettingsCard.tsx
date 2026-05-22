import { useAppStore } from '../../store/appStore'
import { MdRotate90DegreesCcw } from 'react-icons/md'
import { parseLocaleNumber } from '../../utils/numbers'
import type { OriginAnchor } from '../../store/appStore'

type Props = {
  canGenerate: boolean
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

export function PenSettingsCard({ canGenerate, onGenerate }: Props) {
  const settings = useAppStore((state) => state.settings)!
  const imageFile = useAppStore((state) => state.imageFile)
  const selectedColors = useAppStore((state) => state.selectedColors)
  const updateSetting = useAppStore((state) => state.updateSetting)
  const effectiveInfillSpacingMm = settings.customInfillSpacingEnabled ? settings.infillSpacingMm : settings.lineThicknessMm
  const readyToGenerate = Boolean(imageFile && selectedColors.length)
  const rotate90Enabled = Math.abs(settings.rotationDeg - 90) < 1e-9

  return (
    <section className="panel">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Output</div>
          <h2>Pen &amp; G-code</h2>
        </div>
      </div>

      <div className="field-grid compact">
        <label>
          <span>Pen width</span>
          <input
            onChange={(event) => updateSetting('lineThicknessMm', parseLocaleNumber(event.target.value))}
            step="0.01"
            type="number"
            value={settings.lineThicknessMm}
          />
        </label>
      </div>

      <p className="panel-note">
        Infill spacing: {settings.customInfillSpacingEnabled ? `${effectiveInfillSpacingMm.toFixed(2)} mm` : `Auto ${effectiveInfillSpacingMm.toFixed(2)} mm`}.
      </p>

      <div className="generate-row">
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
          <select id="originAnchor" onChange={(event) => updateSetting('originAnchor', event.target.value as OriginAnchor)} value={settings.originAnchor}>
            {ORIGIN_ANCHOR_OPTIONS.map((option) => (
              <option key={option.value} value={option.value}>
                {option.label}
              </option>
            ))}
          </select>
        </label>

        <button
          aria-label="Rotate artwork 90 degrees"
          aria-pressed={rotate90Enabled}
          className={`icon-toggle ${rotate90Enabled ? 'is-active' : ''}`}
          onClick={() => updateSetting('rotationDeg', rotate90Enabled ? 0 : 90)}
          title={rotate90Enabled ? 'Rotation 90° on' : 'Rotation 90° off'}
          type="button"
        >
          <MdRotate90DegreesCcw aria-hidden="true" />
        </button>

        <button className="button primary" disabled={!canGenerate} onClick={onGenerate} type="button">
          {readyToGenerate ? 'Generate G-code' : 'Select a color to generate'}
        </button>
      </div>

      <div className="field-grid compact origin-offset-grid">
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

      {settings.originAnchor === 'custom' ? (
        <p className="panel-note">Custom currently resolves from the artwork center, then applies the manual X/Y offset.</p>
      ) : null}
    </section>
  )
}
