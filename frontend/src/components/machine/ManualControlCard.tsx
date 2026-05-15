import { useAppStore } from '../../store/appStore'

type Props = {
  onJog: (axis: 'X' | 'Y', degrees: number) => void
  onGoHome: () => void
  onPenUp: () => void
  onPenDown: () => void
}

export function ManualControlCard({ onJog, onGoHome, onPenUp, onPenDown }: Props) {
  const settings = useAppStore((state) => state.settings)!
  const updateSetting = useAppStore((state) => state.updateSetting)

  return (
    <section className="panel inset manual-card">
      <div className="panel-heading compact">
        <div>
          <div className="panel-kicker">Manual</div>
          <h2>Manual Control</h2>
        </div>
      </div>

      <div className="jog-pad compact">
        <button className="button jog-button" onClick={() => onJog('Y', settings.yJog || 1)} type="button">
          Y+
        </button>
        <div className="jog-row">
          <button className="button jog-button" onClick={() => onJog('X', -(settings.xJog || 1))} type="button">
            X-
          </button>
          <button className="button jog-home" onClick={onGoHome} type="button">
            X0 / Y0
          </button>
          <button className="button jog-button" onClick={() => onJog('X', settings.xJog || 1)} type="button">
            X+
          </button>
        </div>
        <button className="button jog-button" onClick={() => onJog('Y', -(settings.yJog || 1))} type="button">
          Y-
        </button>
      </div>

      <div className="field-grid compact two">
        <label>
          <span>Step X</span>
          <input onChange={(event) => updateSetting('xJog', Number(event.target.value))} step="0.1" type="number" value={settings.xJog} />
        </label>
        <label>
          <span>Step Y</span>
          <input onChange={(event) => updateSetting('yJog', Number(event.target.value))} step="0.1" type="number" value={settings.yJog} />
        </label>
      </div>

      <div className="stack-row two-up">
        <button className="button" onClick={onPenUp} type="button">
          Pen Up
        </button>
        <button className="button" onClick={onPenDown} type="button">
          Pen Down
        </button>
      </div>

      <details className="details-panel">
        <summary>Servo positions</summary>
        <div className="field-grid compact two">
          <label>
            <span>Servo up</span>
            <input onChange={(event) => updateSetting('penUpS', Number(event.target.value))} type="number" value={settings.penUpS} />
          </label>
          <label>
            <span>Servo down</span>
            <input onChange={(event) => updateSetting('penDownS', Number(event.target.value))} type="number" value={settings.penDownS} />
          </label>
        </div>
      </details>
    </section>
  )
}
