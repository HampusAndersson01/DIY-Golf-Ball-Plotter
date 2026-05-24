import type { MachineState, PreviewPath, PreviewPoint } from '../../api/types'

export const WORLD_BOUNDS = {
  minX: -180,
  maxX: 180,
  minY: -45,
  maxY: 45,
}

export function printableXBounds(maxPrintXSpanDeg: number) {
  const halfSpan = maxPrintXSpanDeg * 0.5
  return {
    minX: -halfSpan,
    maxX: halfSpan,
  }
}

export type PathPhase = 'completed' | 'current' | 'remaining'

export function getProgressPercent(machine: MachineState | null) {
  if (!machine?.progress_total) return 0
  return Math.max(0, Math.min(100, (machine.progress_done / machine.progress_total) * 100))
}

export function classifyPath(path: PreviewPath, machine: MachineState | null): PathPhase {
  if (!machine?.running && !machine?.paused) return 'remaining'
  const line = machine.current_gcode_line
  const start = path.gcode_start_line
  const end = path.gcode_end_line
  if (start == null || end == null) return 'remaining'
  if (line > end) return 'completed'
  if (line >= start && line <= end) return 'current'
  return 'remaining'
}

export function shouldRenderPath(path: PreviewPath, machine: MachineState | null, filter: 'all' | 'progress', showTravel: boolean) {
  if (!showTravel && path.kind === 'travel') return false
  if (filter === 'all') return true
  return classifyPath(path, machine) !== 'remaining'
}

export function getCurrentMarker(path: PreviewPath, machine: MachineState | null): PreviewPoint | null {
  if (!machine || machine.current_path_id !== path.id) return null
  if (!path.points.length) return null
  const pointIndex = Math.max(0, Math.min(machine.current_preview_point_index, path.points.length - 1))
  return path.points[pointIndex] ?? path.points.at(-1) ?? null
}

export function pathColor(kind: string) {
  if (kind === 'fill-wall' || kind === 'outline') return '#f6b756'
  if (kind === 'fill-infill') return '#46c0c6'
  if (kind === 'fill-infill-travel') return '#84cc16'
  if (kind === 'detail-trace') return '#de5eb4'
  if (kind === 'travel') return '#64748b'
  return '#94a3b8'
}

export function phaseOpacity(phase: PathPhase, kind: string) {
  if (phase === 'current') return kind === 'travel' ? 0.94 : 1
  if (phase === 'completed') return kind === 'travel' ? 0.22 : 0.38
  return kind === 'travel' ? 0.32 : 0.88
}

export function phaseStroke(phase: PathPhase, kind: string) {
  if (phase === 'current') return '#fff5a8'
  return pathColor(kind)
}

export function travelRendersAsInfill(path: PreviewPath) {
  return path.kind === 'travel' && path.pen_down === true
}

export function previewVisualKind(path: PreviewPath) {
  if (path.kind === 'fill-infill-travel') return 'fill-infill-travel'
  return travelRendersAsInfill(path) ? 'fill-infill' : path.kind
}

export function previewPathDashed(path: PreviewPath) {
  return path.kind === 'travel' && !travelRendersAsInfill(path)
}

export function derivePreviewBounds(paths: PreviewPath[], maxPrintXSpanDeg: number) {
  const printableBounds = printableXBounds(maxPrintXSpanDeg)
  let minX = WORLD_BOUNDS.minX
  let maxX = WORLD_BOUNDS.maxX
  let minY = WORLD_BOUNDS.minY
  let maxY = WORLD_BOUNDS.maxY
  let found = false

  for (const path of paths) {
    for (const point of path.points) {
      if (!Number.isFinite(point.x) || !Number.isFinite(point.y)) continue
      found = true
      minX = Math.min(minX, point.x)
      maxX = Math.max(maxX, point.x)
      minY = Math.min(minY, point.y)
      maxY = Math.max(maxY, point.y)
    }
  }

  if (!found) {
    return {
      minX: printableBounds.minX,
      maxX: printableBounds.maxX,
      minY: WORLD_BOUNDS.minY,
      maxY: WORLD_BOUNDS.maxY,
    }
  }

  return {
    minX: Math.min(minX - 2, WORLD_BOUNDS.minX),
    maxX: Math.max(maxX + 2, WORLD_BOUNDS.maxX),
    minY: Math.min(minY - 2, WORLD_BOUNDS.minY),
    maxY: Math.max(maxY + 2, WORLD_BOUNDS.maxY),
  }
}
