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
  maxPrintXSpanDeg: number
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
  onZoomChange?: (zoomLabel: string) => void
  zoomLabel: string
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
    <section className="preview-command-center" data-step-anchor="generate">
      <div className="preview-command-center__header">
        <div>
          <div className="panel-kicker">Workspace</div>
          <h2>Preview &amp; Live Visualization</h2>
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
      </div>

      <div className="preview-command-center__body">
        <div className="preview-surface">
          <div className="preview-grid-overlay" />
          <div className="preview-stage-frame">
            <CurrentLineOverlay currentPath={currentPath} machine={props.machine} />
            {props.previewMode === '2d' ? (
              <Toolpath2DView
                ref={twoDRef}
                filter={props.progressFilter}
                machine={props.machine}
                maxPrintXSpanDeg={props.maxPrintXSpanDeg}
                paths={props.paths}
                showTravel={props.showTravel}
              />
            ) : (
              <Ball3DView
                ref={threeDRef}
                filter={props.progressFilter}
                maxPrintXSpanDeg={props.maxPrintXSpanDeg}
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

            <div className="preview-floating-legend">
              <ToolpathLegend />
            </div>

            <div className="preview-floating-zoom" aria-hidden>
              <div className="zoom-pill">
                <strong>{props.zoomLabel}</strong>
                <small>Wheel zoom · drag pan · double-click fit · F = fit · R = reset</small>
              </div>
            </div>

            {/* Floating zoom pill removed — zoom is shown in the canvas meta and via the top toolbar state. */}

            {/* Bottom toolbar removed to avoid duplicate view controls. Top `PreviewToolbar` contains Printer/Front/Fit/Reset. */}

            <div className="preview-status-strip">
              <div className="status-left">
                <span>X: {formatCoordinate(props.machine?.current_position_x)}</span>
                <span>Y: {formatCoordinate(props.machine?.current_position_y)}</span>
                <span>Z: 0.000</span>
              </div>

              <div className="status-right">Live Visualization Engine v2.1</div>
            </div>
          </div>
        </div>
      </div>
    </section>
  )
}



function formatCoordinate(value: number | undefined) {
  if (value == null || !Number.isFinite(value)) return '--'
  return value.toFixed(3)
}
