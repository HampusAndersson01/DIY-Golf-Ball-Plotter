import type { JobSummary } from '../../api/types'

type Props = {
  summary: JobSummary | null
  generationDurationMs: number | null
}

export function JobSummaryPanel({ summary, generationDurationMs }: Props) {
  if (!summary) {
    return (
      <section className="panel">
        <div className="panel-heading">
          <div>
            <div className="panel-kicker">Summary</div>
            <h2>Job Summary</h2>
          </div>
        </div>
        <p className="panel-copy muted">Generate G-code to review runtime, path counts, and print size before calibration and run.</p>
      </section>
    )
  }

  return (
    <section className="panel">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Summary</div>
          <h2>Job Summary</h2>
        </div>
      </div>

      <div className="summary-grid">
        <div><span>Image size</span><strong>{summary.image_size}</strong></div>
        <div><span>Selected colors</span><strong>{summary.selected_colors.length}</strong></div>
        <div><span>Mask pixels</span><strong>{summary.mask_pixel_count}</strong></div>
        <div><span>Raw G-code lines</span><strong>{summary.gcode_line_count}</strong></div>
        <div><span>Streamable lines</span><strong>{summary.streamable_gcode_line_count ?? summary.estimated_runtime_breakdown?.streamableGcodeLines ?? '--'}</strong></div>
        <div><span>Estimated runtime</span><strong>{formatDuration(summary.estimated_runtime_seconds)}</strong></div>
        <div><span>Motion estimate</span><strong>{formatDuration(summary.estimated_runtime_breakdown?.estimatedMotionSeconds ?? 0)}</strong></div>
        <div><span>Streaming overhead</span><strong>{formatDuration(summary.estimated_runtime_breakdown?.estimatedStreamingOverheadSeconds ?? 0)}</strong></div>
        <div><span>Pen lifts</span><strong>{summary.pen_lift_count}</strong></div>
        <div><span>Walls</span><strong>{summary.wall_path_count}</strong></div>
        <div><span>Infill</span><strong>{summary.infill_path_count}</strong></div>
        <div><span>Detail</span><strong>{summary.detail_trace_path_count}</strong></div>
        <div><span>Travel</span><strong>{summary.travel_path_count}</strong></div>
        <div><span>Components</span><strong>{summary.component_count}</strong></div>
        <div><span>Actual runtime</span><strong>{summary.actual_runtime_seconds != null ? formatDuration(summary.actual_runtime_seconds) : '--'}</strong></div>
        <div><span>Actual vs estimate</span><strong>{summary.actual_vs_estimated_ratio != null ? `${summary.actual_vs_estimated_ratio.toFixed(1)}x` : '--'}</strong></div>
        <div><span>Generate time</span><strong>{formatGeneration(generationDurationMs)}</strong></div>
      </div>
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
