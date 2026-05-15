import { useAppStore } from '../../store/appStore'

type Props = {
  onJog: (axis: 'X' | 'Y', degrees: number) => void
  onGoHome: () => void
}

export function JogControls({ onJog, onGoHome }: Props) {
  const settings = useAppStore((state) => state.settings)!

  return (
    <section className="panel inset">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Motion</div>
          <h2>Jog Controls</h2>
        </div>
      </div>
      <div className="jog-pad">
        <button className="button jog-button" onClick={() => onJog('Y', settings.yJog || 1)} type="button">
          Up
        </button>
        <div className="jog-row">
          <button className="button jog-button" onClick={() => onJog('X', -(settings.xJog || 1))} type="button">
            Left
          </button>
          <button className="button jog-home" onClick={onGoHome} type="button">
            Home
          </button>
          <button className="button jog-button" onClick={() => onJog('X', settings.xJog || 1)} type="button">
            Right
          </button>
        </div>
        <button className="button jog-button" onClick={() => onJog('Y', -(settings.yJog || 1))} type="button">
          Down
        </button>
      </div>
      <div className="jog-footer">
        <div className="metric-tile">
          <span>Step X</span>
          <strong>{settings.xJog.toFixed(2)}°</strong>
        </div>
        <div className="metric-tile">
          <span>Step Y</span>
          <strong>{settings.yJog.toFixed(2)}°</strong>
        </div>
      </div>
    </section>
  )
}
