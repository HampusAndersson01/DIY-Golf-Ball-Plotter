import type { JobSummary } from '../../api/types'

type Props = {
  summary: JobSummary | null
  generationDurationMs: number | null
}

export function JobSummaryPanel({ summary, generationDurationMs }: Props) {
  if (!summary) {
    return (
      <section className="inspector-card inspector-card--note">
        <div className="inspector-card__header">
          <div>
            <span className="panel-kicker">Summary</span>
            <strong>Job Summary</strong>
          </div>
        </div>
        <p>Generate G-code to review runtime, path counts, and print size before calibration and run.</p>
      </section>
    )
  }

  return (
    <section className="inspector-summary-shell">
      <div className="inspector-card-grid">
        <section className="inspector-card inspector-card--image">
          <div className="inspector-card__header">
            <div>
              <span className="panel-kicker">Input image</span>
              <strong>{summary.image_size}</strong>
            </div>
            <div className="inspector-icon">▣</div>
          </div>
          <div className="inspector-thumbnail-placeholder">Selected image</div>
        </section>

        <section className="inspector-card inspector-card--metric"><span className="panel-kicker">G-code lines</span><strong>{summary.gcode_line_count}</strong></section>
        <section className="inspector-card inspector-card--metric"><span className="panel-kicker">Estimated runtime</span><strong>{formatDuration(summary.estimated_runtime_seconds)}</strong></section>
        <section className="inspector-card inspector-card--metric"><span className="panel-kicker">Pen lifts</span><strong>{summary.pen_lift_count}</strong></section>
      </div>

      <details className="details-panel inspector-details">
        <summary>Detailed metrics</summary>
        <div className="summary-grid summary-grid--compact">
          <div><span>Selected colors</span><strong>{summary.selected_colors.length}</strong></div>
          <div><span>Mask pixels</span><strong>{summary.mask_pixel_count}</strong></div>
          <div><span>Streamable lines</span><strong>{summary.streamable_gcode_line_count ?? summary.estimated_runtime_breakdown?.streamableGcodeLines ?? '--'}</strong></div>
          <div><span>Motion estimate</span><strong>{formatDuration(summary.estimated_runtime_breakdown?.estimatedMotionSeconds ?? 0)}</strong></div>
          <div><span>Streaming overhead</span><strong>{formatDuration(summary.estimated_runtime_breakdown?.estimatedStreamingOverheadSeconds ?? 0)}</strong></div>
          <div><span>Short-segment overhead</span><strong>{formatDuration(summary.estimated_runtime_breakdown?.estimatedShortSegmentOverheadSeconds ?? 0)}</strong></div>
          <div><span>Walls</span><strong>{summary.wall_path_count}</strong></div>
          <div><span>Infill</span><strong>{summary.infill_path_count}</strong></div>
          <div><span>Detail</span><strong>{summary.detail_trace_path_count}</strong></div>
          <div><span>Travel</span><strong>{summary.travel_path_count}</strong></div>
          <div><span>Components</span><strong>{summary.component_count}</strong></div>
          <div><span>Actual runtime</span><strong>{summary.actual_runtime_seconds != null ? formatDuration(summary.actual_runtime_seconds) : '--'}</strong></div>
          <div><span>Actual vs estimate</span><strong>{summary.actual_vs_estimated_ratio != null ? `${summary.actual_vs_estimated_ratio.toFixed(1)}x` : '--'}</strong></div>
          <div><span>Generate time</span><strong>{formatGeneration(generationDurationMs)}</strong></div>
        </div>
      </details>
    </section>
  )
}

function formatDuration(totalSeconds: number) {
  const seconds = Math.max(0, Math.round(totalSeconds))
  const hours = Math.floor(seconds / 3600)
  const minutes = Math.floor((seconds % 3600) / 60)
  const remainder = seconds % 60
  return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(remainder).padStart(2, '0')}`
}

function formatGeneration(totalMs: number | null) {
  if (!totalMs) return '--'
  return `${(totalMs / 1000).toFixed(2)}s`
}
