import { useAppStore } from '../../store/appStore'
import { parseLocaleNumber } from '../../utils/numbers'

type Props = {
  canGenerate: boolean
  onGenerate: () => void
}

export function PenSettingsCard({ canGenerate, onGenerate }: Props) {
  const settings = useAppStore((state) => state.settings)!
  const imageFile = useAppStore((state) => state.imageFile)
  const selectedColors = useAppStore((state) => state.selectedColors)
  const updateSetting = useAppStore((state) => state.updateSetting)
  const effectiveInfillSpacingMm = settings.customInfillSpacingEnabled ? settings.infillSpacingMm : settings.lineThicknessMm
  const readyToGenerate = Boolean(imageFile && selectedColors.length)

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
        Normal printing uses safe defaults automatically. Infill spacing: {settings.customInfillSpacingEnabled ? `${effectiveInfillSpacingMm.toFixed(2)} mm` : `Auto = ${effectiveInfillSpacingMm.toFixed(2)} mm`}.
      </p>

      <p className="panel-copy muted">
        Workflow: import image, analyze colors, pick the printable color, then generate G-code. Placement, infill tuning, cleanup, and other slicer controls are in Advanced only.
      </p>

      <button className="button primary" disabled={!canGenerate} onClick={onGenerate} type="button">
        {readyToGenerate ? 'Generate G-code' : 'Select a color to generate'}
      </button>
    </section>
  )
}
