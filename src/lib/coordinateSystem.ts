import { type Vector2, vec } from './vector'

/**
 * Maps between world space (meters, Y-up, origin configurable) and screen space
 * (pixels, Y-down, origin top-left) so simulation math can stay in conventional
 * math coordinates while rendering to a canvas.
 */
export interface CoordinateSystem {
  /** Pixels per world-space meter. */
  scale: number
  /** World-space point that maps to the center of the canvas. */
  origin: Vector2
  width: number
  height: number
}

export const createCoordinateSystem = (
  width: number,
  height: number,
  scale = 40,
  origin: Vector2 = vec(0, 0),
): CoordinateSystem => ({ scale, origin, width, height })

export const worldToScreen = (cs: CoordinateSystem, p: Vector2): Vector2 => ({
  x: cs.width / 2 + (p.x - cs.origin.x) * cs.scale,
  y: cs.height / 2 - (p.y - cs.origin.y) * cs.scale,
})

export const screenToWorld = (cs: CoordinateSystem, p: Vector2): Vector2 => ({
  x: (p.x - cs.width / 2) / cs.scale + cs.origin.x,
  y: -(p.y - cs.height / 2) / cs.scale + cs.origin.y,
})

/** Scales a world-space length (e.g. a vector magnitude or radius) into pixels. */
export const worldLengthToScreen = (cs: CoordinateSystem, length: number): number =>
  length * cs.scale
