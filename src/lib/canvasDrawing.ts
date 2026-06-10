import type { Vector2 } from './vector'

interface ArrowStyle {
  color: string
  lineWidth?: number
  headLength?: number
}

/** Draws a straight arrow (shaft + triangular head) between two screen-space points. */
export const drawArrow = (
  ctx: CanvasRenderingContext2D,
  from: Vector2,
  to: Vector2,
  { color, lineWidth = 2, headLength = 8 }: ArrowStyle,
) => {
  const dx = to.x - from.x
  const dy = to.y - from.y
  const length = Math.hypot(dx, dy)
  if (length < 1) return

  const angle = Math.atan2(dy, dx)
  const headAngle = Math.PI / 7

  ctx.strokeStyle = color
  ctx.fillStyle = color
  ctx.lineWidth = lineWidth

  ctx.beginPath()
  ctx.moveTo(from.x, from.y)
  ctx.lineTo(to.x, to.y)
  ctx.stroke()

  ctx.beginPath()
  ctx.moveTo(to.x, to.y)
  ctx.lineTo(to.x - headLength * Math.cos(angle - headAngle), to.y - headLength * Math.sin(angle - headAngle))
  ctx.lineTo(to.x - headLength * Math.cos(angle + headAngle), to.y - headLength * Math.sin(angle + headAngle))
  ctx.closePath()
  ctx.fill()
}
