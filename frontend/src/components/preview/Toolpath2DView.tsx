import { forwardRef, useEffect, useEffectEvent, useImperativeHandle, useMemo, useRef, useState } from 'react'

import type { MachineState, PreviewPath } from '../../api/types'
import { PanZoomCanvas } from './PanZoomCanvas'
import { WORLD_BOUNDS, classifyPath, derivePreviewBounds, getCurrentMarker, pathColor, phaseOpacity, phaseStroke, shouldRenderPath } from './previewMath'

export type Toolpath2DHandle = {
  fit: () => void
  reset: () => void
}

type Props = {
  paths: PreviewPath[]
  machine: MachineState | null
  filter: 'all' | 'progress'
  showTravel: boolean
}

export const Toolpath2DView = forwardRef<Toolpath2DHandle, Props>(function Toolpath2DView(
  { paths, machine, filter, showTravel },
  ref,
) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const controllerRef = useRef<PanZoomCanvas | null>(null)
  const [zoomLabel, setZoomLabel] = useState('100%')

  const visiblePaths = useMemo(
    () => paths.filter((path) => shouldRenderPath(path, machine, filter, showTravel)),
    [filter, machine, paths, showTravel],
  )

  const renderCanvas = useEffectEvent((ctx: CanvasRenderingContext2D, engine: PanZoomCanvas) => {
    const { width, height } = engine.getViewState()
    ctx.fillStyle = '#f4efe6'
    ctx.fillRect(0, 0, width * window.devicePixelRatio, height * window.devicePixelRatio)

    engine.applyTransform(ctx)
    drawGrid(ctx, engine)
    for (const path of visiblePaths) {
      const phase = classifyPath(path, machine)
      const points = path.points.filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y))
      if (points.length < 2) continue
      ctx.beginPath()
      ctx.moveTo(points[0].x, -points[0].y)
      for (let index = 1; index < points.length; index += 1) {
        ctx.lineTo(points[index].x, -points[index].y)
      }
      ctx.setLineDash(path.kind === 'travel' ? [3 / engine.getScale(), 2 / engine.getScale()] : [])
      ctx.lineCap = 'round'
      ctx.lineJoin = 'round'
      ctx.lineWidth = (phase === 'current' ? 2.4 : 1.6) / engine.getScale()
      ctx.strokeStyle = hexWithAlpha(phaseStroke(phase, path.kind), phaseOpacity(phase, path.kind))
      ctx.stroke()

      const marker = getCurrentMarker(path, machine)
      if (marker) {
        ctx.fillStyle = '#fff5a8'
        ctx.beginPath()
        ctx.arc(marker.x, -marker.y, 4.4 / engine.getScale(), 0, Math.PI * 2)
        ctx.fill()
        ctx.strokeStyle = '#0f172a'
        ctx.lineWidth = 1 / engine.getScale()
        ctx.stroke()
      }
    }
    ctx.setLineDash([])
  })

  useImperativeHandle(ref, () => ({
    fit: () => controllerRef.current?.fitToView(),
    reset: () => controllerRef.current?.resetView(),
  }))

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const controller = new PanZoomCanvas(canvas, {
      minScale: 0.2,
      maxScale: 72,
      onChange: (view) => setZoomLabel(`${Math.round(view.scale * 100)}%`),
    })
    controllerRef.current = controller
    controller.setRenderer((ctx, engine) => renderCanvas(ctx, engine))
    controller.setContentBounds(flipBoundsY(derivePreviewBounds(paths)))
    controller.fitToView()

    const observer = new ResizeObserver(() => controller.resize(true))
    observer.observe(canvas)
    return () => {
      observer.disconnect()
      controller.destroy()
      controllerRef.current = null
    }
  }, [paths])

  useEffect(() => {
    const controller = controllerRef.current
    if (!controller) return
    controller.setContentBounds(flipBoundsY(derivePreviewBounds(paths)))
    controller.setRenderer((ctx, engine) => renderCanvas(ctx, engine))
  }, [machine, paths, showTravel, filter, visiblePaths])

  return (
    <div className="toolpath-canvas-wrap">
      <canvas ref={canvasRef} className="toolpath-canvas" />
      <div className="canvas-meta">
        <span>Wheel zoom, drag pan, double-click fit, `F` fit, `R` reset</span>
        <strong>{zoomLabel}</strong>
      </div>
    </div>
  )
})

function drawGrid(ctx: CanvasRenderingContext2D, engine: PanZoomCanvas) {
  const stepX = 45
  const stepY = 15
  ctx.strokeStyle = 'rgba(100, 116, 139, 0.35)'
  ctx.lineWidth = 1 / engine.getScale()
  for (let x = WORLD_BOUNDS.minX; x <= WORLD_BOUNDS.maxX; x += stepX) {
    ctx.beginPath()
    ctx.moveTo(x, -WORLD_BOUNDS.maxY)
    ctx.lineTo(x, -WORLD_BOUNDS.minY)
    ctx.stroke()
  }
  for (let y = WORLD_BOUNDS.minY; y <= WORLD_BOUNDS.maxY; y += stepY) {
    ctx.beginPath()
    ctx.moveTo(WORLD_BOUNDS.minX, -y)
    ctx.lineTo(WORLD_BOUNDS.maxX, -y)
    ctx.stroke()
  }

  ctx.strokeStyle = '#254660'
  ctx.lineWidth = 1.2 / engine.getScale()
  ctx.strokeRect(WORLD_BOUNDS.minX, -WORLD_BOUNDS.maxY, WORLD_BOUNDS.maxX - WORLD_BOUNDS.minX, WORLD_BOUNDS.maxY - WORLD_BOUNDS.minY)
  ctx.strokeStyle = '#1d6a86'
  ctx.beginPath()
  ctx.moveTo(0, -WORLD_BOUNDS.maxY)
  ctx.lineTo(0, -WORLD_BOUNDS.minY)
  ctx.moveTo(WORLD_BOUNDS.minX, 0)
  ctx.lineTo(WORLD_BOUNDS.maxX, 0)
  ctx.stroke()

  ctx.fillStyle = '#3c5466'
  ctx.font = `${11 / engine.getScale()}px ui-sans-serif`
  ctx.fillText('X rotation -180', WORLD_BOUNDS.minX + 4, -WORLD_BOUNDS.minY + 6)
  ctx.fillText('X +180', WORLD_BOUNDS.maxX - 24, -WORLD_BOUNDS.minY + 6)
  ctx.fillText('Y +45', WORLD_BOUNDS.minX + 4, -WORLD_BOUNDS.maxY + 5)
  ctx.fillText('Y -45', WORLD_BOUNDS.minX + 4, -WORLD_BOUNDS.minY - 3)
  ctx.fillStyle = pathColor('outline')
  ctx.beginPath()
  ctx.arc(0, 0, 1.8 / engine.getScale(), 0, Math.PI * 2)
  ctx.fill()
}

function flipBoundsY(bounds: { minX: number; maxX: number; minY: number; maxY: number }) {
  return {
    minX: bounds.minX,
    maxX: bounds.maxX,
    minY: -bounds.maxY,
    maxY: -bounds.minY,
  }
}

function hexWithAlpha(hex: string, alpha: number) {
  const sanitized = hex.replace('#', '')
  const full = sanitized.length === 3 ? sanitized.split('').map((char) => `${char}${char}`).join('') : sanitized
  const value = parseInt(full, 16)
  const r = (value >> 16) & 255
  const g = (value >> 8) & 255
  const b = value & 255
  return `rgba(${r}, ${g}, ${b}, ${alpha})`
}
