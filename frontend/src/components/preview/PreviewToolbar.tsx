import type { PreviewMode } from '../../store/appStore'

type Props = {
  previewMode: PreviewMode
  progressFilter: 'all' | 'progress'
  showTravel: boolean
  showPenWidth: boolean
  showMask: boolean
  onModeChange: (mode: PreviewMode) => void
  onFilterChange: (value: 'all' | 'progress') => void
  onShowTravel: (value: boolean) => void
  onShowPenWidth: (value: boolean) => void
  onShowMask: (value: boolean) => void
  onFit: () => void
  onReset: () => void
  onViewPreset: (preset: 'printer' | 'front') => void
}

export function PreviewToolbar({
  previewMode,
  progressFilter,
  showTravel,
  showPenWidth,
  showMask,
  onModeChange,
  onFilterChange,
  onShowTravel,
  onShowPenWidth,
  onShowMask,
  onFit,
  onReset,
  onViewPreset,
}: Props) {
  return (
    <div className="preview-toolbar">
      <div className="segmented">
        <button className={previewMode === '2d' ? 'active' : ''} onClick={() => onModeChange('2d')} type="button">
          2D
        </button>
        <button className={previewMode === '3d' ? 'active' : ''} onClick={() => onModeChange('3d')} type="button">
          3D
        </button>
      </div>

      <div className="toolbar-group">
        <div className="segmented compact">
          <button className={progressFilter === 'all' ? 'active' : ''} onClick={() => onFilterChange('all')} type="button">
            All
          </button>
          <button className={progressFilter === 'progress' ? 'active' : ''} onClick={() => onFilterChange('progress')} type="button">
            Progress
          </button>
        </div>

        <label className="toggle">
          <input checked={showTravel} onChange={(event) => onShowTravel(event.target.checked)} type="checkbox" />
          <span>Travel</span>
        </label>

        <label className="toggle">
          <input checked={showPenWidth} onChange={(event) => onShowPenWidth(event.target.checked)} type="checkbox" />
          <span>Pen width</span>
        </label>

        <label className="toggle">
          <input checked={showMask} onChange={(event) => onShowMask(event.target.checked)} type="checkbox" />
          <span>Show mask</span>
        </label>
      </div>

      <div className="toolbar-group">
        <button className="button subtle" onClick={() => onViewPreset('printer')} type="button">
          Printer
        </button>
        <button className="button subtle" onClick={() => onViewPreset('front')} type="button">
          Front
        </button>
        <button className="button subtle" onClick={onFit} type="button">
          Fit
        </button>
        <button className="button subtle" onClick={onReset} type="button">
          Reset
        </button>
      </div>
    </div>
  )
}
