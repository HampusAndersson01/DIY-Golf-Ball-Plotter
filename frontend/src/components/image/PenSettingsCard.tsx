import { useAppStore } from '../../store/appStore'
import { parseLocaleNumber } from '../../utils/numbers'

type Props = {
  canGenerate: boolean
  onGenerate: () => void
}

export function PenSettingsCard({ canGenerate, onGenerate }: Props) {
  const settings = useAppStore((state) => state.settings)!
  const updateSetting = useAppStore((state) => state.updateSetting)
  const effectiveInfillSpacingMm = settings.customInfillSpacingEnabled ? settings.infillSpacingMm : settings.lineThicknessMm

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
        Infill spacing: {settings.customInfillSpacingEnabled ? `${effectiveInfillSpacingMm.toFixed(2)} mm` : `Auto = ${effectiveInfillSpacingMm.toFixed(2)} mm`}
      </p>

      <button className="button primary" disabled={!canGenerate} onClick={onGenerate} type="button">
        Generate G-code
      </button>
    </section>
  )
}
