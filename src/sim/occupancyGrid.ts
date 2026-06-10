import type { Vector2 } from '@/lib/vector'
import type { Obstacle } from '@/sim/forces'
import { simulateLidar } from '@/sim/sensors'

// Log-odds update magnitudes per observation
const L_OCC = 0.9    // confident hit → cell is occupied
const L_FREE = 0.4   // ray passed through → cell is free (lower: false-negatives more common)
const L_CLAMP = 3.5  // saturate at ~97% confidence in either direction

// Grid covers ±WORLD_EXTENT metres from the origin in both axes.
const WORLD_EXTENT = 20

// Perfect-sensor settings used when Phase 12 sensor mode is off.
// Denser/longer than the default LiDAR for thorough mapping coverage.
const MAPPING_RAYS = 72
const MAPPING_RANGE = 15

export interface OccupancyGrid {
  logOdds: Float32Array  // row-major: index = row * cols + col
  cols: number
  rows: number
  cellSize: number   // metres per cell
  originX: number    // world X of column 0
  originY: number    // world Y of row 0 (lowest Y value)
}

export interface OccupancyGridStats {
  total: number
  mapped: number
  free: number
  occupied: number
  coveragePct: number
}

export interface GridConfig {
  cellSize: number
}

export const DEFAULT_GRID_CONFIG: GridConfig = { cellSize: 0.3 }

export function createGrid(config: GridConfig): OccupancyGrid {
  const n = Math.ceil((2 * WORLD_EXTENT) / config.cellSize)
  return {
    logOdds: new Float32Array(n * n), // initialised to 0 = unknown
    cols: n,
    rows: n,
    cellSize: config.cellSize,
    originX: -WORLD_EXTENT,
    originY: -WORLD_EXTENT,
  }
}

export function clearGrid(grid: OccupancyGrid): void {
  grid.logOdds.fill(0)
}

function worldToCell(grid: OccupancyGrid, wx: number, wy: number): [number, number] | null {
  const col = Math.floor((wx - grid.originX) / grid.cellSize)
  const row = Math.floor((wy - grid.originY) / grid.cellSize)
  if (col < 0 || col >= grid.cols || row < 0 || row >= grid.rows) return null
  return [col, row]
}

// Bresenham's line — visits every grid cell from (c0,r0) to (c1,r1) inclusive.
// Assumes both endpoints are in bounds; does not check.
function traceLine(
  c0: number, r0: number,
  c1: number, r1: number,
  visit: (col: number, row: number, isEndpoint: boolean) => void,
): void {
  const dc = Math.abs(c1 - c0)
  const dr = Math.abs(r1 - r0)
  const sc = c0 < c1 ? 1 : -1
  const sr = r0 < r1 ? 1 : -1
  let err = dc - dr
  let c = c0
  let r = r0

  for (;;) {
    const isEnd = c === c1 && r === r1
    visit(c, r, isEnd)
    if (isEnd) break
    const e2 = 2 * err
    if (e2 > -dr) { err -= dr; c += sc }
    if (e2 < dc) { err += dc; r += sr }
  }
}

/**
 * Bayesian log-odds update from a set of LiDAR hits (Phase 12 data).
 *
 * For each ray:
 *   • Cells along the ray before the endpoint → L_FREE (beam traversed = no obstacle)
 *   • Endpoint cell (if obstacle hit)         → L_OCC  (beam stopped = obstacle here)
 */
export function updateGridFromHits(
  grid: OccupancyGrid,
  robotPos: Vector2,
  hits: Array<{ worldPoint: Vector2; hitObstacle: boolean; dropped: boolean }>,
): void {
  const startCell = worldToCell(grid, robotPos.x, robotPos.y)
  if (!startCell) return
  const [sc, sr] = startCell

  for (const hit of hits) {
    if (hit.dropped) continue
    const endCell = worldToCell(grid, hit.worldPoint.x, hit.worldPoint.y)
    if (!endCell) continue
    const [ec, er] = endCell

    traceLine(sc, sr, ec, er, (col, row, isEndpoint) => {
      const idx = row * grid.cols + col
      const delta = isEndpoint && hit.hitObstacle ? L_OCC : -L_FREE
      grid.logOdds[idx] = Math.max(-L_CLAMP, Math.min(L_CLAMP, grid.logOdds[idx] + delta))
    })
  }
}

/**
 * Noiseless internal scan — used when Phase 12 sensor mode is off so the
 * robot can still build a map with perfect knowledge. Reuses simulateLidar
 * with 0 noise and 0 dropout to avoid duplicating the ray-casting logic.
 */
export function updateGridPerfect(
  grid: OccupancyGrid,
  robotPos: Vector2,
  obstacles: Obstacle[],
): void {
  const hits = simulateLidar(robotPos, 0, obstacles, {
    enabled: true,
    numRays: MAPPING_RAYS,
    range: MAPPING_RANGE,
    rangeNoise: 0,
    dropoutRate: 0,
  })
  updateGridFromHits(grid, robotPos, hits)
}

/** Map-coverage statistics — fast Float32Array scan (~0.1 ms for 17 k cells). */
export function gridStats(grid: OccupancyGrid, threshold = 0.5): OccupancyGridStats {
  let free = 0
  let occupied = 0
  const total = grid.cols * grid.rows
  const lo = grid.logOdds
  for (let i = 0; i < total; i++) {
    if (lo[i] > threshold) occupied++
    else if (lo[i] < -threshold) free++
  }
  const mapped = free + occupied
  return { total, mapped, free, occupied, coveragePct: (mapped / total) * 100 }
}
