/** A 2D vector in world-space (meters). Used for positions, velocities, and forces alike. */
export interface Vector2 {
  x: number
  y: number
}

export const vec = (x: number, y: number): Vector2 => ({ x, y })

export const zero = (): Vector2 => ({ x: 0, y: 0 })

export const add = (a: Vector2, b: Vector2): Vector2 => ({ x: a.x + b.x, y: a.y + b.y })

export const sub = (a: Vector2, b: Vector2): Vector2 => ({ x: a.x - b.x, y: a.y - b.y })

export const scale = (v: Vector2, s: number): Vector2 => ({ x: v.x * s, y: v.y * s })

export const dot = (a: Vector2, b: Vector2): number => a.x * b.x + a.y * b.y

export const magnitude = (v: Vector2): number => Math.hypot(v.x, v.y)

export const distance = (a: Vector2, b: Vector2): number => magnitude(sub(a, b))

/** Returns a unit vector in the direction of `v`, or the zero vector if `v` has ~zero length. */
export const normalize = (v: Vector2): Vector2 => {
  const m = magnitude(v)
  if (m < 1e-9) return zero()
  return scale(v, 1 / m)
}

export const limit = (v: Vector2, max: number): Vector2 => {
  const m = magnitude(v)
  return m > max ? scale(v, max / m) : v
}

export const sum = (vectors: Vector2[]): Vector2 => vectors.reduce(add, zero())
