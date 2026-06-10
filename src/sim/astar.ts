import type { Vector2 } from '@/lib/vector'
import { distance } from '@/lib/vector'
import type { Obstacle } from './forces'

export interface AStarResult {
  path: Vector2[]
  planningTimeMs: number
  nodesExplored: number
  success: boolean
}

// Planning grid covers any canvas viewport at 40px/m up to ~1200×900px.
const BOUNDS = { minX: -20, maxX: 20, minY: -20, maxY: 20 }
const RES = 0.4     // meters per cell
const INFLATION = 0.4  // extra clearance added beyond obstacle radius

const COLS = Math.ceil((BOUNDS.maxX - BOUNDS.minX) / RES) // 100
const ROWS = Math.ceil((BOUNDS.maxY - BOUNDS.minY) / RES) // 100

const worldToGrid = (p: Vector2): [number, number] => [
  Math.round((p.x - BOUNDS.minX) / RES - 0.5),
  Math.round((p.y - BOUNDS.minY) / RES - 0.5),
]

const gridToWorld = (cx: number, cy: number): Vector2 => ({
  x: BOUNDS.minX + (cx + 0.5) * RES,
  y: BOUNDS.minY + (cy + 0.5) * RES,
})

const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v))

// 8-connected grid directions with their travel costs.
const DIRS: [dx: number, dy: number, cost: number][] = [
  [1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1],
  [1, 1, Math.SQRT2], [-1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, -1, Math.SQRT2],
]

// ── Min-heap keyed by f-score ─────────────────────────────────────────────────
// Stores pairs [f, cellIdx] packed into a Float64Array for cache friendliness.
class MinHeap {
  private d: Float64Array
  private n = 0
  constructor(cap: number) { this.d = new Float64Array(cap * 2) }

  push(f: number, idx: number) {
    let i = this.n++
    this.d[i * 2] = f; this.d[i * 2 + 1] = idx
    while (i > 0) {
      const p = (i - 1) >> 1
      if (this.d[p * 2] <= this.d[i * 2]) break
      this._swap(i, p); i = p
    }
  }

  pop(): [f: number, idx: number] {
    const f = this.d[0], idx = this.d[1]
    if (--this.n > 0) {
      this.d[0] = this.d[this.n * 2]; this.d[1] = this.d[this.n * 2 + 1]
      let i = 0
      while (true) {
        const l = 2 * i + 1, r = l + 1
        let s = i
        if (l < this.n && this.d[l * 2] < this.d[s * 2]) s = l
        if (r < this.n && this.d[r * 2] < this.d[s * 2]) s = r
        if (s === i) break
        this._swap(i, s); i = s
      }
    }
    return [f, idx]
  }

  get empty() { return this.n === 0 }

  private _swap(a: number, b: number) {
    let t = this.d[a * 2]; this.d[a * 2] = this.d[b * 2]; this.d[b * 2] = t
    t = this.d[a * 2 + 1]; this.d[a * 2 + 1] = this.d[b * 2 + 1]; this.d[b * 2 + 1] = t
  }
}

// ── Segment–circle collision test used by the path smoother ──────────────────
const segmentHitsCircle = (a: Vector2, b: Vector2, center: Vector2, radius: number): boolean => {
  const abx = b.x - a.x, aby = b.y - a.y
  const acx = center.x - a.x, acy = center.y - a.y
  const len2 = abx * abx + aby * aby
  const t = len2 < 1e-12 ? 0 : clamp((acx * abx + acy * aby) / len2, 0, 1)
  const dx = a.x + t * abx - center.x, dy = a.y + t * aby - center.y
  return dx * dx + dy * dy < radius * radius
}

// Greedy string-pulling: repeatedly try to connect the current waypoint to the
// farthest visible successor, skipping grid staircases that aren't real turns.
const smoothPath = (path: Vector2[], obstacles: Obstacle[]): Vector2[] => {
  if (path.length <= 2) return path
  const out = [path[0]]
  let i = 0
  while (i < path.length - 1) {
    let j = path.length - 1
    while (j > i + 1) {
      const clear = obstacles.every((o) => !segmentHitsCircle(path[i], path[j], o.position, o.radius + INFLATION))
      if (clear) break
      j--
    }
    out.push(path[j])
    i = j
  }
  return out
}

// ── Main planner ──────────────────────────────────────────────────────────────
export const planAStar = (start: Vector2, goal: Vector2, obstacles: Obstacle[]): AStarResult => {
  const t0 = performance.now()

  // Pre-rasterize blocked cells using a bounding box per obstacle.
  const blocked = new Uint8Array(COLS * ROWS)
  for (const obs of obstacles) {
    const reach = obs.radius + INFLATION
    const x0 = clamp(Math.floor((obs.position.x - reach - BOUNDS.minX) / RES), 0, COLS - 1)
    const x1 = clamp(Math.ceil((obs.position.x + reach - BOUNDS.minX) / RES), 0, COLS - 1)
    const y0 = clamp(Math.floor((obs.position.y - reach - BOUNDS.minY) / RES), 0, ROWS - 1)
    const y1 = clamp(Math.ceil((obs.position.y + reach - BOUNDS.minY) / RES), 0, ROWS - 1)
    for (let cy = y0; cy <= y1; cy++) {
      for (let cx = x0; cx <= x1; cx++) {
        if (distance(gridToWorld(cx, cy), obs.position) < reach) blocked[cy * COLS + cx] = 1
      }
    }
  }

  const [sc, sr] = worldToGrid(start)
  const [gc, gr] = worldToGrid(goal)
  const startIdx = clamp(sr, 0, ROWS - 1) * COLS + clamp(sc, 0, COLS - 1)
  const goalIdx = clamp(gr, 0, ROWS - 1) * COLS + clamp(gc, 0, COLS - 1)

  const gScore = new Float32Array(COLS * ROWS).fill(Infinity)
  const cameFrom = new Int32Array(COLS * ROWS).fill(-1) // parent cell index

  const heap = new MinHeap(COLS * ROWS * 3)
  gScore[startIdx] = 0
  const [gsc, gsr] = [clamp(sc, 0, COLS - 1), clamp(sr, 0, ROWS - 1)]
  const h0 = distance(gridToWorld(gsc, gsr), goal)
  heap.push(h0, startIdx)

  let nodesExplored = 0
  let found = false

  while (!heap.empty) {
    const [, curIdx] = heap.pop()
    const curG = gScore[curIdx]
    if (curIdx === goalIdx) { found = true; break }
    nodesExplored++

    const cy = Math.floor(curIdx / COLS), cx = curIdx - cy * COLS
    for (const [dx, dy, cost] of DIRS) {
      const nx = cx + dx, ny = cy + dy
      if (nx < 0 || nx >= COLS || ny < 0 || ny >= ROWS) continue
      const nIdx = ny * COLS + nx
      if (blocked[nIdx]) continue
      const tentG = curG + cost * RES
      if (tentG < gScore[nIdx]) {
        gScore[nIdx] = tentG
        cameFrom[nIdx] = curIdx
        heap.push(tentG + distance(gridToWorld(nx, ny), goal), nIdx)
      }
    }
  }

  // Reconstruct and smooth path.
  const path: Vector2[] = []
  if (found) {
    let idx = goalIdx
    while (idx !== startIdx) {
      const cy = Math.floor(idx / COLS), cx = idx - cy * COLS
      path.push(gridToWorld(cx, cy))
      idx = cameFrom[idx]
    }
    path.push(gridToWorld(gsc, gsr))
    path.reverse()
    path[0] = { ...start }
    path[path.length - 1] = { ...goal }
    return { path: smoothPath(path, obstacles), planningTimeMs: performance.now() - t0, nodesExplored, success: true }
  }

  return { path: [], planningTimeMs: performance.now() - t0, nodesExplored, success: false }
}
