import { useCallback, useEffect, useMemo, useRef } from 'react'

import type { MachineState, PreviewPath } from '../../api/types'
import type { PreviewMode, ViewPreset } from '../../store/appStore'
import type { Ball3DHandle } from './Ball3DView'
import { Ball3DView } from './Ball3DView'
import { CurrentLineOverlay } from './CurrentLineOverlay'
import { PreviewToolbar } from './PreviewToolbar'
import type { Toolpath2DHandle } from './Toolpath2DView'
import { Toolpath2DView } from './Toolpath2DView'
import { ToolpathLegend } from './ToolpathLegend'

type Props = {
  paths: PreviewPath[]
  machine: MachineState | null
  previewMode: PreviewMode
  progressFilter: 'all' | 'progress'
  showTravel: boolean
  showCompare: boolean
  imagePreviewUrl: string | null
  maskPreviewUrl: string | null
  viewPreset: ViewPreset
  onPreviewMode: (mode: PreviewMode) => void
  onProgressFilter: (filter: 'all' | 'progress') => void
  onShowTravel: (show: boolean) => void
  onShowCompare: (show: boolean) => void
  onViewPreset: (preset: ViewPreset) => void
}

export function PreviewWorkspace(props: Props) {
  const twoDRef = useRef<Toolpath2DHandle | null>(null)
  const threeDRef = useRef<Ball3DHandle | null>(null)
  const currentPath = useMemo(
    () => props.paths.find((path) => path.id === props.machine?.current_path_id) ?? null,
    [props.machine?.current_path_id, props.paths],
  )

  const fit = useCallback(() => {
    if (props.previewMode === '2d') twoDRef.current?.fit()
    else threeDRef.current?.fit()
  }, [props.previewMode])

  const reset = useCallback(() => {
    if (props.previewMode === '2d') twoDRef.current?.reset()
    else threeDRef.current?.reset()
  }, [props.previewMode])

  useEffect(() => {
    const onFit = () => fit()
    const onReset = () => reset()
    window.addEventListener('preview-fit', onFit)
    window.addEventListener('preview-reset', onReset)
    return () => {
      window.removeEventListener('preview-fit', onFit)
      window.removeEventListener('preview-reset', onReset)
    }
  }, [fit, reset])

  return (
    <section className="panel preview-panel" data-step-anchor="generate">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Workspace</div>
          <h2>Preview &amp; Live Visualization</h2>
        </div>
      </div>

      <PreviewToolbar
        onFilterChange={props.onProgressFilter}
        onFit={fit}
        onModeChange={props.onPreviewMode}
        onReset={reset}
        onShowCompare={props.onShowCompare}
        onShowTravel={props.onShowTravel}
        onViewPreset={props.onViewPreset}
        previewMode={props.previewMode}
        progressFilter={props.progressFilter}
        showCompare={props.showCompare}
        showTravel={props.showTravel}
      />

      <ToolpathLegend />

      <div className="preview-surface">
        <CurrentLineOverlay currentPath={currentPath} machine={props.machine} />
        {props.previewMode === '2d' ? (
          <Toolpath2DView ref={twoDRef} filter={props.progressFilter} machine={props.machine} paths={props.paths} showTravel={props.showTravel} />
        ) : (
          <Ball3DView
            ref={threeDRef}
            filter={props.progressFilter}
            machine={props.machine}
            paths={props.paths}
            preset={props.viewPreset}
            showTravel={props.showTravel}
          />
        )}

        {props.showCompare ? (
          <aside className="compare-popover">
            <div>
              <span>Original</span>
              {props.imagePreviewUrl ? <img alt="Original upload" src={props.imagePreviewUrl} /> : <div className="compare-empty">No source image</div>}
            </div>
            <div>
              <span>Mask</span>
              {props.maskPreviewUrl ? <img alt="Selected mask" src={props.maskPreviewUrl} /> : <div className="compare-empty">No generated mask</div>}
            </div>
          </aside>
        ) : null}
      </div>
    </section>
  )
}
