type Bounds = {
  minX: number
  maxX: number
  minY: number
  maxY: number
}

type ViewState = {
  scale: number
  offsetX: number
  offsetY: number
  width: number
  height: number
  dpr: number
}

type Options = {
  minScale?: number
  maxScale?: number
  onChange?: (view: ViewState) => void
}

export class PanZoomCanvas {
  private canvas: HTMLCanvasElement
  private ctx: CanvasRenderingContext2D
  private minScale: number
  private maxScale: number
  private onChange: (view: ViewState) => void
  private contentBounds: Bounds | null = null
  private drawFn: ((ctx: CanvasRenderingContext2D, controller: PanZoomCanvas) => void) | null = null
  private dpr = window.devicePixelRatio || 1
  private width = 1
  private height = 1
  private scale = 1
  private offsetX = 0
  private offsetY = 0
  private pointerId: number | null = null
  private lastX = 0
  private lastY = 0
  private raf = 0
  private defaultView: { scale: number; offsetX: number; offsetY: number } | null = null

  constructor(canvas: HTMLCanvasElement, options: Options = {}) {
    this.canvas = canvas
    this.ctx = canvas.getContext('2d') as CanvasRenderingContext2D
    this.minScale = options.minScale ?? 0.15
    this.maxScale = options.maxScale ?? 64
    this.onChange = options.onChange ?? (() => {})

    this.handleWheel = this.handleWheel.bind(this)
    this.handlePointerDown = this.handlePointerDown.bind(this)
    this.handlePointerMove = this.handlePointerMove.bind(this)
    this.handlePointerUp = this.handlePointerUp.bind(this)
    this.handleDoubleClick = this.handleDoubleClick.bind(this)

    canvas.addEventListener('wheel', this.handleWheel, { passive: false })
    canvas.addEventListener('pointerdown', this.handlePointerDown)
    canvas.addEventListener('pointermove', this.handlePointerMove)
    canvas.addEventListener('pointerup', this.handlePointerUp)
    canvas.addEventListener('pointercancel', this.handlePointerUp)
    canvas.addEventListener('lostpointercapture', this.handlePointerUp)
    canvas.addEventListener('dblclick', this.handleDoubleClick)
    this.resize(false)
  }

  destroy() {
    this.canvas.removeEventListener('wheel', this.handleWheel)
    this.canvas.removeEventListener('pointerdown', this.handlePointerDown)
    this.canvas.removeEventListener('pointermove', this.handlePointerMove)
    this.canvas.removeEventListener('pointerup', this.handlePointerUp)
    this.canvas.removeEventListener('pointercancel', this.handlePointerUp)
    this.canvas.removeEventListener('lostpointercapture', this.handlePointerUp)
    this.canvas.removeEventListener('dblclick', this.handleDoubleClick)
  }

  setContentBounds(bounds: Bounds | null) {
    this.contentBounds = bounds
  }

  setRenderer(drawFn: (ctx: CanvasRenderingContext2D, controller: PanZoomCanvas) => void) {
    this.drawFn = drawFn
    this.requestDraw()
  }

  getViewState(): ViewState {
    return {
      scale: this.scale,
      offsetX: this.offsetX,
      offsetY: this.offsetY,
      width: this.width,
      height: this.height,
      dpr: this.dpr,
    }
  }

  getScale() {
    return this.scale
  }

  resize(preserveCenter = true) {
    const previousCenter = preserveCenter ? this.screenToWorld(this.width / 2, this.height / 2) : null
    const rect = this.canvas.getBoundingClientRect()
    this.width = Math.max(300, Math.round(rect.width || 300))
    this.height = Math.max(300, Math.round(rect.height || 300))
    this.dpr = window.devicePixelRatio || 1
    this.canvas.width = Math.round(this.width * this.dpr)
    this.canvas.height = Math.round(this.height * this.dpr)

    if (previousCenter) {
      this.offsetX = this.width / 2 - previousCenter.x * this.scale
      this.offsetY = this.height / 2 - previousCenter.y * this.scale
    }

    this.requestDraw()
    this.onChange(this.getViewState())
  }

  fitToView(padding = 26) {
    if (!this.contentBounds) {
      this.scale = 1
      this.offsetX = this.width / 2
      this.offsetY = this.height / 2
      this.defaultView = { scale: this.scale, offsetX: this.offsetX, offsetY: this.offsetY }
      this.requestDraw()
      this.onChange(this.getViewState())
      return
    }

    const contentWidth = Math.max(0.001, this.contentBounds.maxX - this.contentBounds.minX)
    const contentHeight = Math.max(0.001, this.contentBounds.maxY - this.contentBounds.minY)
    const nextScale = Math.max(
      this.minScale,
      Math.min(this.maxScale, Math.min((this.width - padding * 2) / contentWidth, (this.height - padding * 2) / contentHeight)),
    )
    const centerX = (this.contentBounds.minX + this.contentBounds.maxX) / 2
    const centerY = (this.contentBounds.minY + this.contentBounds.maxY) / 2
    this.scale = nextScale
    this.offsetX = this.width / 2 - centerX * nextScale
    this.offsetY = this.height / 2 - centerY * nextScale
    this.defaultView = { scale: this.scale, offsetX: this.offsetX, offsetY: this.offsetY }
    this.requestDraw()
    this.onChange(this.getViewState())
  }

  resetView() {
    if (!this.defaultView) {
      this.fitToView()
      return
    }
    this.scale = this.defaultView.scale
    this.offsetX = this.defaultView.offsetX
    this.offsetY = this.defaultView.offsetY
    this.requestDraw()
    this.onChange(this.getViewState())
  }

  zoomAt(factor: number, x: number, y: number) {
    const world = this.screenToWorld(x, y)
    this.scale = clamp(this.scale * factor, this.minScale, this.maxScale)
    this.offsetX = x - world.x * this.scale
    this.offsetY = y - world.y * this.scale
    this.requestDraw()
    this.onChange(this.getViewState())
  }

  worldToScreen(x: number, y: number) {
    return {
      x: x * this.scale + this.offsetX,
      y: y * this.scale + this.offsetY,
    }
  }

  screenToWorld(x: number, y: number) {
    return {
      x: (x - this.offsetX) / this.scale,
      y: (y - this.offsetY) / this.scale,
    }
  }

  applyTransform(ctx = this.ctx) {
    ctx.setTransform(this.dpr * this.scale, 0, 0, this.dpr * this.scale, this.dpr * this.offsetX, this.dpr * this.offsetY)
  }

  requestDraw() {
    if (this.raf) return
    this.raf = window.requestAnimationFrame(() => {
      this.raf = 0
      this.ctx.setTransform(1, 0, 0, 1, 0, 0)
      this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)
      this.drawFn?.(this.ctx, this)
    })
  }

  private handleWheel(event: WheelEvent) {
    event.preventDefault()
    const rect = this.canvas.getBoundingClientRect()
    const x = event.clientX - rect.left
    const y = event.clientY - rect.top
    if (Math.abs(event.deltaX) > 0 && Math.abs(event.deltaY) < 5) {
      this.offsetX -= event.deltaX
      this.requestDraw()
      this.onChange(this.getViewState())
      return
    }
    this.zoomAt(Math.exp(-event.deltaY * 0.0015), x, y)
  }

  private handlePointerDown(event: PointerEvent) {
    if (event.button !== 0) return
    event.preventDefault()
    this.pointerId = event.pointerId
    this.lastX = event.clientX
    this.lastY = event.clientY
    this.canvas.setPointerCapture(event.pointerId)
    this.canvas.classList.add('is-dragging')
  }

  private handlePointerMove(event: PointerEvent) {
    if (this.pointerId !== event.pointerId) return
    event.preventDefault()
    const deltaX = event.clientX - this.lastX
    const deltaY = event.clientY - this.lastY
    this.lastX = event.clientX
    this.lastY = event.clientY
    this.offsetX += deltaX
    this.offsetY += deltaY
    this.requestDraw()
    this.onChange(this.getViewState())
  }

  private handlePointerUp(event: PointerEvent) {
    if (this.pointerId !== event.pointerId) return
    this.pointerId = null
    this.canvas.classList.remove('is-dragging')
    if (this.canvas.hasPointerCapture(event.pointerId)) {
      this.canvas.releasePointerCapture(event.pointerId)
    }
  }

  private handleDoubleClick(event: MouseEvent) {
    event.preventDefault()
    this.fitToView()
  }
}

function clamp(value: number, min: number, max: number) {
  return Math.min(max, Math.max(min, value))
}
