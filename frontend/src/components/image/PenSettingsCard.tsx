import { useAppStore } from '../../store/appStore'

type Props = {
  canGenerate: boolean
  onGenerate: () => void
}

export function PenSettingsCard({ canGenerate, onGenerate }: Props) {
  const settings = useAppStore((state) => state.settings)!
  const updateSetting = useAppStore((state) => state.updateSetting)

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
          <input onChange={(event) => updateSetting('lineThicknessMm', Number(event.target.value))} step="0.01" type="number" value={settings.lineThicknessMm} />
        </label>
      </div>

      <button className="button primary" disabled={!canGenerate} onClick={onGenerate} type="button">
        Generate G-code
      </button>
    </section>
  )
}
