import type { Vector2 } from '@/lib/vector'
import { distance } from '@/lib/vector'
import type { Obstacle } from './forces'

export interface RRTResult {
  path: Vector2[]
  // Edges of the exploration tree — capped for rendering performance.
  tree: Array<{ from: Vector2; to: Vector2 }>
  planningTimeMs: number
  nodesExplored: number
  success: boolean
}

const BOUNDS = { minX: -20, maxX: 20, minY: -20, maxY: 20 }
const STEP_SIZE = 0.6    // meters per tree extension
const GOAL_BIAS = 0.12   // probability of sampling the goal directly
const MAX_ITER = 3000
const INFLATION = 0.3    // extra clearance beyond obstacle radius
const GOAL_CONNECT_DIST = STEP_SIZE  // connect to goal when this close
const TREE_RENDER_CAP = 600  // max edges sent to the canvas

const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v))

const rand = (): Vector2 => ({
  x: BOUNDS.minX + Math.random() * (BOUNDS.maxX - BOUNDS.minX),
  y: BOUNDS.minY + Math.random() * (BOUNDS.maxY - BOUNDS.minY),
})

const normalize2 = (dx: number, dy: number): [number, number] => {
  const len = Math.sqrt(dx * dx + dy * dy)
  return len < 1e-9 ? [0, 0] : [dx / len, dy / len]
}

// Minimum distance from segment a→b to circle center, compared to radius.
const segmentHitsCircle = (ax: number, ay: number, bx: number, by: number, cx: number, cy: number, r: number): boolean => {
  const abx = bx - ax, aby = by - ay
  const acx = cx - ax, acy = cy - ay
  const len2 = abx * abx + aby * aby
  const t = len2 < 1e-12 ? 0 : clamp((acx * abx + acy * aby) / len2, 0, 1)
  const ex = ax + t * abx - cx, ey = ay + t * aby - cy
  return ex * ex + ey * ey < r * r
}

const segmentClear = (ax: number, ay: number, bx: number, by: number, obstacles: Obstacle[]): boolean => {
  for (const o of obstacles) {
    if (segmentHitsCircle(ax, ay, bx, by, o.position.x, o.position.y, o.radius + INFLATION)) return false
  }
  return true
}

export const planRRT = (start: Vector2, goal: Vector2, obstacles: Obstacle[]): RRTResult => {
  const t0 = performance.now()

  // Node storage as parallel arrays — faster than array-of-objects for hot nearest-neighbor loop.
  const nx: number[] = [start.x]
  const ny: number[] = [start.y]
  const parent: number[] = [-1]

  const treeEdges: Array<{ from: Vector2; to: Vector2 }> = []
  let success = false
  let goalParent = -1

  for (let iter = 0; iter < MAX_ITER; iter++) {
    // Sample: goal bias or random
    const sample = Math.random() < GOAL_BIAS ? goal : rand()

    // Nearest node (linear scan — fast enough for 3000 nodes in V8)
    let nearestIdx = 0
    let nearestDist = distance({ x: nx[0], y: ny[0] }, sample)
    for (let i = 1; i < nx.length; i++) {
      const d = distance({ x: nx[i], y: ny[i] }, sample)
      if (d < nearestDist) { nearestDist = d; nearestIdx = i }
      // Early exit: nothing inside a cell smaller than STEP_SIZE/2 can be nearer
      if (nearestDist < STEP_SIZE * 0.5) break
    }

    // Steer toward sample
    const dx = sample.x - nx[nearestIdx], dy = sample.y - ny[nearestIdx]
    const [ndx, ndy] = normalize2(dx, dy)
    const step = Math.min(STEP_SIZE, nearestDist)
    const newX = nx[nearestIdx] + ndx * step
    const newY = ny[nearestIdx] + ndy * step

    if (!segmentClear(nx[nearestIdx], ny[nearestIdx], newX, newY, obstacles)) continue

    const newIdx = nx.length
    nx.push(newX); ny.push(newY); parent.push(nearestIdx)
    if (treeEdges.length < TREE_RENDER_CAP) {
      treeEdges.push({ from: { x: nx[nearestIdx], y: ny[nearestIdx] }, to: { x: newX, y: newY } })
    }

    // Check connection to goal
    const distToGoal = distance({ x: newX, y: newY }, goal)
    if (distToGoal < GOAL_CONNECT_DIST && segmentClear(newX, newY, goal.x, goal.y, obstacles)) {
      goalParent = newIdx
      success = true
      break
    }
  }

  // Reconstruct path from goal back to start.
  const path: Vector2[] = []
  if (success) {
    path.push({ ...goal })
    let cur = goalParent
    while (cur !== -1) {
      path.push({ x: nx[cur], y: ny[cur] })
      cur = parent[cur]
    }
    path.reverse()
    path[0] = { ...start }
  }

  return {
    path,
    tree: treeEdges,
    planningTimeMs: performance.now() - t0,
    nodesExplored: nx.length,
    success,
  }
}
