import type { Vector2 } from '@/lib/vector'
import { distance } from '@/lib/vector'
import type { Obstacle } from './forces'

export const computePathLength = (path: Vector2[]): number => {
  let len = 0
  for (let i = 1; i < path.length; i++) len += distance(path[i - 1], path[i])
  return len
}

// Sum of absolute angle changes (in radians) at each interior waypoint.
// 0 = perfectly straight; higher = more tortuous.
export const computeSmoothness = (path: Vector2[]): number => {
  if (path.length < 3) return 0
  let total = 0
  for (let i = 1; i < path.length - 1; i++) {
    const ax = path[i].x - path[i - 1].x, ay = path[i].y - path[i - 1].y
    const bx = path[i + 1].x - path[i].x, by = path[i + 1].y - path[i].y
    const la = Math.sqrt(ax * ax + ay * ay), lb = Math.sqrt(bx * bx + by * by)
    if (la < 1e-9 || lb < 1e-9) continue
    const cosA = Math.max(-1, Math.min(1, (ax * bx + ay * by) / (la * lb)))
    total += Math.acos(cosA)
  }
  return total
}

// Minimum distance from any path waypoint to the nearest obstacle surface.
export const computeMinClearance = (path: Vector2[], obstacles: Obstacle[]): number => {
  if (obstacles.length === 0) return Infinity
  let minClear = Infinity
  for (const p of path) {
    for (const o of obstacles) {
      const clear = Math.max(distance(p, o.position) - o.radius, 0)
      if (clear < minClear) minClear = clear
    }
  }
  return minClear
}
