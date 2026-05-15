import { useAppStore } from '../../store/appStore'

type Props = {
  onPenUp: () => void
  onPenDown: () => void
  onGoHome: () => void
}

export function PenControls({ onPenUp, onPenDown, onGoHome }: Props) {
  const settings = useAppStore((state) => state.settings)!
  const updateSetting = useAppStore((state) => state.updateSetting)

  return (
    <section className="panel inset">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Pen</div>
          <h2>Pen Controls</h2>
        </div>
      </div>
      <div className="stack-row two-up">
        <button className="button" onClick={onPenUp} type="button">
          Pen Up
        </button>
        <button className="button" onClick={onPenDown} type="button">
          Pen Down
        </button>
      </div>
      <div className="field-grid compact two">
        <label>
          <span>Servo up (us)</span>
          <input onChange={(event) => updateSetting('penUpS', Number(event.target.value))} type="number" value={settings.penUpS} />
        </label>
        <label>
          <span>Servo down (us)</span>
          <input onChange={(event) => updateSetting('penDownS', Number(event.target.value))} type="number" value={settings.penDownS} />
        </label>
      </div>
      <button className="button subtle" onClick={onGoHome} type="button">
        Go X0 / Y0
      </button>
    </section>
  )
}
