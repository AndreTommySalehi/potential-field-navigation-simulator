import { useEffect, useRef, type MouseEvent } from 'react'
import type { IntegrationMethod, SimulationState } from '@/sim/simulationEngine'
import { attractivePotentialAt, computeForces, DEFAULT_PARAMS, fieldForceAt, potentialAt, type Obstacle, type SimulationParams } from '@/sim/forces'
import type { AlgorithmComparisonState, ComparisonTrace, LocalMinimaDiagnostics, RecoveryInfo, RecoveryMethod } from '@/hooks/useSimulationLoop'
import type { LidarHit, SensorConfig } from '@/sim/sensors'
import type { OccupancyGrid } from '@/sim/occupancyGrid'
import type { UncertaintyStats } from '@/sim/uncertainty'
import type { SLAMState } from '@/sim/slam'
import {
  createCoordinateSystem,
  worldToScreen,
  screenToWorld,
  worldLengthToScreen,
  type CoordinateSystem,
} from '@/lib/coordinateSystem'
import { drawArrow } from '@/lib/canvasDrawing'
import { distance, dot, magnitude, normalize, scale, sub, type Vector2 } from '@/lib/vector'

interface SimulationCanvasProps {
  state: SimulationState
  params?: SimulationParams
  showVectorField?: boolean
  // Spacing between sampled field arrows, in meters — lower is denser.
  fieldSpacing?: number
  showHeatmap?: boolean
  // Side length of each heatmap cell, in meters — lower is sharper (and pricier).
  heatmapResolution?: number
  // The other two integrators' trajectories, run in parallel for comparison —
  // drawn as faint ghost trails so their drift from the active one is visible.
  comparisonTraces?: ComparisonTrace[]
  // Phase 8 — when the gradient has gone flat away from the goal long enough
  // to call it a trap, this carries where it happened so it can be labeled.
  localMinima?: LocalMinimaDiagnostics
  // Phase 9 — recovery state: active method, temp waypoint, stats.
  recovery?: RecoveryInfo
  recoveryMethod?: RecoveryMethod
  // Phase 11 — A* and RRT plans + path-follower robots
  algorithmComparison?: AlgorithmComparisonState
  // Show integration-method ghost robots on the canvas (default off — useful for
  // the Integration panel deep-dive but clutters normal use).
  showComparisonTraces?: boolean
  // Phase 12 — sensor simulation overlay
  sensorConfig?: SensorConfig
  lidarHits?: LidarHit[]
  // Phase 13 — occupancy grid overlay
  showOccupancyGrid?: boolean
  occupancyGrid?: OccupancyGrid
  // Phase 14 — uncertainty ellipse
  showUncertainty?: boolean
  uncertaintyStats?: UncertaintyStats
  uncertaintySigmaLevel?: number
  // Phase 15 — SLAM Lite overlay
  showSLAM?: boolean
  slamState?: SLAMState | null
  pixelsPerMeter?: number
  onCanvasClick?: (point: Vector2) => void
}

const ROBOT_RADIUS_M = 0.3
const GOAL_RADIUS_M = 0.25
const GHOST_RADIUS_M = ROBOT_RADIUS_M * 0.7
const GRID_SPACING_M = 1

// Both arrow scales are "pixels per newton" — kept separate because the field
// overlay's arrows get capped to their grid cell and don't need much headroom.
const FORCE_ARROW_SCALE = 18
const MAX_FORCE_ARROW_PX = 140
const FIELD_ARROW_SCALE = 6
const FIELD_ARROW_CELL_RATIO = 0.42

// One color per integration method, matching IntegrationPanel's legend so a
// trail on the canvas and a row in the sidebar read as the same thing.
const TRACE_COLORS: Record<IntegrationMethod, [number, number, number]> = {
  euler: [251, 146, 60],
  'semi-implicit-euler': [232, 121, 249],
  rk4: [52, 211, 153],
}

// Below this, ResizeObserver tends to notice its own canvas writes and loop.
const RESIZE_EPSILON_PX = 0.5

// Reusable offscreen canvas + pixel buffers for the occupancy grid overlay.
// Recreated only when grid dimensions change (i.e. cell-size slider moves).
let _occCanvas: HTMLCanvasElement | null = null
let _occBuf: Uint8ClampedArray | null = null
let _occInfluence: Uint8Array | null = null  // 1 where a free cell is inside Q*

const drawOccupancyGrid = (
  ctx: CanvasRenderingContext2D,
  cs: CoordinateSystem,
  grid: OccupancyGrid,
  influenceRadius: number,
) => {
  const { cols, rows } = grid
  if (!_occCanvas || _occCanvas.width !== cols || _occCanvas.height !== rows) {
    _occCanvas = document.createElement('canvas')
    _occCanvas.width = cols
    _occCanvas.height = rows
    _occBuf = new Uint8ClampedArray(cols * rows * 4)
    _occInfluence = new Uint8Array(cols * rows)
  }
  const d = _occBuf!
  const inf = _occInfluence!

  const L_CLAMP = 3.5
  const THRESHOLD = 0.2
  // Influence radius in grid cells (float — use squared comparison to avoid sqrt).
  const infCells = influenceRadius / grid.cellSize
  const infCells2 = infCells * infCells
  const infCeil = Math.ceil(infCells)

  // Pass 1 — spread influence zone outward from every confidently-occupied cell.
  // Typical occupied cell count is O(obstacle_perimeters / cellSize) ≈ 20–200,
  // so the inner loop rarely exceeds a few thousand writes per frame.
  inf.fill(0)
  for (let row = 0; row < rows; row++) {
    for (let col = 0; col < cols; col++) {
      if (grid.logOdds[row * cols + col] <= THRESHOLD) continue
      for (let dr = -infCeil; dr <= infCeil; dr++) {
        for (let dc = -infCeil; dc <= infCeil; dc++) {
          if (dc * dc + dr * dr > infCells2) continue
          const r2 = row + dr
          const c2 = col + dc
          if (r2 >= 0 && r2 < rows && c2 >= 0 && c2 < cols) inf[r2 * cols + c2] = 1
        }
      }
    }
  }

  // Pass 2 — fill the pixel buffer.
  for (let gridRow = 0; gridRow < rows; gridRow++) {
    // Grid row 0 = world bottom; image row 0 = screen top — flip Y.
    const imageRow = rows - 1 - gridRow
    for (let col = 0; col < cols; col++) {
      const lo = grid.logOdds[gridRow * cols + col]
      const absLo = Math.abs(lo)
      const px = (imageRow * cols + col) * 4

      if (absLo < THRESHOLD) {
        d[px + 3] = 0  // transparent = unknown
        continue
      }

      const conf = Math.min((absLo - THRESHOLD) / (L_CLAMP - THRESHOLD), 1)

      if (lo > 0) {
        // Occupied: red
        d[px] = 239; d[px + 1] = 68; d[px + 2] = 68
        d[px + 3] = Math.round((0.35 + 0.5 * conf) * 255)
      } else if (inf[gridRow * cols + col]) {
        // Free but inside an obstacle's influence radius: amber caution zone.
        // Mirrors the dashed Q* ring drawn around each obstacle so the map and
        // the canvas annotation tell the same story.
        d[px] = 245; d[px + 1] = 158; d[px + 2] = 11
        d[px + 3] = Math.round((0.15 + 0.2 * conf) * 255)
      } else {
        // Confidently free: dark slate
        d[px] = 51; d[px + 1] = 65; d[px + 2] = 85
        d[px + 3] = Math.round((0.12 + 0.45 * conf) * 255)
      }
    }
  }

  const offCtx = _occCanvas.getContext('2d')!
  offCtx.putImageData(new ImageData(d, cols, rows), 0, 0)

  const topLeft = worldToScreen(cs, { x: grid.originX, y: grid.originY + rows * grid.cellSize })
  const bottomRight = worldToScreen(cs, { x: grid.originX + cols * grid.cellSize, y: grid.originY })
  ctx.save()
  ctx.imageSmoothingEnabled = false
  ctx.drawImage(_occCanvas, topLeft.x, topLeft.y, bottomRight.x - topLeft.x, bottomRight.y - topLeft.y)
  ctx.restore()
}

/**
 * Draw an n-sigma confidence ellipse around the robot's estimated position.
 *
 * The ellipse is derived from the 2×2 position covariance: semi-axes
 * = n * sqrt(eigenvalue), oriented along the principal eigenvectors.
 * In canvas coordinates +y is down, so the world-space angle is negated.
 */
const drawUncertaintyEllipse = (
  ctx: CanvasRenderingContext2D,
  cs: CoordinateSystem,
  robotPos: Vector2,
  stats: UncertaintyStats,
  sigmaLevel: number,
) => {
  if (stats.sigma1 < 0.01) return  // essentially zero uncertainty — nothing to draw
  const center = worldToScreen(cs, robotPos)
  const rx = worldLengthToScreen(cs, stats.sigma1 * sigmaLevel)
  const ry = worldLengthToScreen(cs, stats.sigma2 * sigmaLevel)
  if (rx < 0.5) return

  // World angle → canvas angle: negate because canvas +y is down vs world +y up.
  const rotation = -stats.angle

  ctx.save()
  // Subtle fill to show the probability mass region
  ctx.beginPath()
  ctx.ellipse(center.x, center.y, Math.max(rx, 0.5), Math.max(ry, 0.5), rotation, 0, Math.PI * 2)
  ctx.fillStyle = 'rgba(99,102,241,0.07)'
  ctx.fill()
  // Dashed border
  ctx.beginPath()
  ctx.ellipse(center.x, center.y, Math.max(rx, 0.5), Math.max(ry, 0.5), rotation, 0, Math.PI * 2)
  ctx.strokeStyle = 'rgba(129,140,248,0.8)'
  ctx.lineWidth = 1.5
  ctx.setLineDash([6, 4])
  ctx.stroke()
  ctx.setLineDash([])
  ctx.restore()
}

// ─── Phase 15: SLAM Lite Overlay ──────────────────────────────────────────────
//
// Three visual layers:
//   1. Error line   — dashed yellow line from SLAM estimated pos to true pos.
//                     Length = localization error. Spikes during motion, shrinks
//                     at loop-closure corrections.
//   2. Landmarks    — yellow diamonds sized by sqrt(sigma2) (uncertainty).
//                     Wide = freshly added; tight = converged.
//   3. Ghost robot  — yellow circle at estimated pos; distinct from the blue APF
//                     robot (true pos). When SLAM is working well they overlap;
//                     drift pulls them apart.
//   4. Closure ring — pulsing ring at the last loop-closure site (fades over 3 s).

const SLAM_YELLOW = 'rgba(253,224,71,'    // yellow-300
const SLAM_GHOST_RADIUS_M = ROBOT_RADIUS_M * 0.75

const drawSLAMOverlay = (
  ctx: CanvasRenderingContext2D,
  cs: CoordinateSystem,
  slam: SLAMState,
  truePos: Vector2,
  simTime: number,
) => {
  ctx.save()

  const estScreen = worldToScreen(cs, slam.estPos)
  const trueScreen = worldToScreen(cs, truePos)

  // 1. Error line from estimated pos to true pos
  if (slam.posError > 0.05) {
    ctx.beginPath()
    ctx.moveTo(estScreen.x, estScreen.y)
    ctx.lineTo(trueScreen.x, trueScreen.y)
    ctx.strokeStyle = SLAM_YELLOW + '0.45)'
    ctx.lineWidth = 1.5
    ctx.setLineDash([4, 5])
    ctx.stroke()
    ctx.setLineDash([])
  }

  // 2. Landmarks: uncertainty circle + diamond centrepoint
  for (const lm of slam.landmarks) {
    const lmScreen = worldToScreen(cs, lm.pos)
    const sigmaPx = worldLengthToScreen(cs, Math.sqrt(lm.sigma2))

    // Translucent uncertainty disc
    if (sigmaPx > 1.5) {
      ctx.beginPath()
      ctx.arc(lmScreen.x, lmScreen.y, Math.max(sigmaPx, 2), 0, Math.PI * 2)
      ctx.fillStyle = SLAM_YELLOW + '0.06)'
      ctx.fill()
      ctx.strokeStyle = SLAM_YELLOW + '0.25)'
      ctx.lineWidth = 1
      ctx.stroke()
    }

    // Diamond marker — size proportional to confidence (shrinks as landmark converges)
    const r = Math.max(3, Math.min(sigmaPx * 0.6, 9))
    ctx.fillStyle = SLAM_YELLOW + '0.75)'
    ctx.strokeStyle = SLAM_YELLOW + '0.95)'
    ctx.lineWidth = 1.5
    ctx.beginPath()
    ctx.moveTo(lmScreen.x, lmScreen.y - r)
    ctx.lineTo(lmScreen.x + r, lmScreen.y)
    ctx.lineTo(lmScreen.x, lmScreen.y + r)
    ctx.lineTo(lmScreen.x - r, lmScreen.y)
    ctx.closePath()
    ctx.fill()
    ctx.stroke()
  }

  // 3. Loop-closure flash — pulsing ring at last closure site, fades over 3 s
  if (slam.lastClosurePos && slam.lastClosureTime !== null) {
    const age = simTime - slam.lastClosureTime
    if (age < 3.0) {
      const fadeAlpha = (1 - age / 3.0) * 0.85
      const pulse = (Math.sin(simTime * 8) + 1) * 0.5
      const closureScreen = worldToScreen(cs, slam.lastClosurePos)
      const baseR = worldLengthToScreen(cs, ROBOT_RADIUS_M * 2)
      ctx.beginPath()
      ctx.arc(closureScreen.x, closureScreen.y, baseR + pulse * 6, 0, Math.PI * 2)
      ctx.strokeStyle = SLAM_YELLOW + `${fadeAlpha})`
      ctx.lineWidth = 2
      ctx.stroke()
    }
  }

  // 4. Ghost robot at SLAM estimated position (drawn before true robot so blue is on top)
  const ghostR = worldLengthToScreen(cs, SLAM_GHOST_RADIUS_M)
  ctx.beginPath()
  ctx.arc(estScreen.x, estScreen.y, ghostR, 0, Math.PI * 2)
  ctx.fillStyle = SLAM_YELLOW + '0.55)'
  ctx.fill()
  ctx.strokeStyle = SLAM_YELLOW + '0.85)'
  ctx.lineWidth = 1.5
  ctx.stroke()

  ctx.restore()
}

export const SimulationCanvas = ({
  state,
  params = DEFAULT_PARAMS,
  showVectorField = false,
  fieldSpacing = 1,
  showHeatmap = false,
  heatmapResolution = 0.6,
  comparisonTraces = [],
  showComparisonTraces = false,
  localMinima,
  recovery,
  recoveryMethod = 'none',
  algorithmComparison,
  sensorConfig,
  lidarHits = [],
  showOccupancyGrid = false,
  occupancyGrid,
  showUncertainty = false,
  uncertaintyStats,
  uncertaintySigmaLevel = 2,
  showSLAM = false,
  slamState = null,
  pixelsPerMeter = 40,
  onCanvasClick,
}: SimulationCanvasProps) => {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const containerRef = useRef<HTMLDivElement>(null)
  const coordsRef = useRef<CoordinateSystem>(createCoordinateSystem(0, 0, pixelsPerMeter))
  const stateRef = useRef(state)
  const paramsRef = useRef(params)
  const tracesRef = useRef(comparisonTraces)
  const localMinimaRef = useRef(localMinima)
  const recoveryRef = useRef(recovery)
  const recoveryMethodRef = useRef(recoveryMethod)
  const algoRef = useRef(algorithmComparison)
  const sensorConfigRef = useRef(sensorConfig)
  const lidarHitsRef = useRef(lidarHits)
  const slamStateRef = useRef(slamState)
  stateRef.current = state
  paramsRef.current = params
  tracesRef.current = comparisonTraces
  localMinimaRef.current = localMinima
  recoveryRef.current = recovery
  recoveryMethodRef.current = recoveryMethod
  algoRef.current = algorithmComparison
  sensorConfigRef.current = sensorConfig
  lidarHitsRef.current = lidarHits
  slamStateRef.current = slamState

  const draw = () => {
    const ctx = canvasRef.current?.getContext('2d')
    if (!ctx) return
    const cs = coordsRef.current
    const current = stateRef.current
    const liveParams = paramsRef.current
    ctx.fillStyle = '#0b0f17'
    ctx.fillRect(0, 0, cs.width, cs.height)
    if (showHeatmap) drawPotentialHeatmap(ctx, cs, current, liveParams, heatmapResolution)
    drawGrid(ctx, cs)
    // Phase 13 — occupancy grid shows the robot's probabilistic map estimate.
    // Drawn after the background grid so grid lines stay visible on top of it,
    // and before obstacles so true circles render above the map cells.
    if (showOccupancyGrid && occupancyGrid) drawOccupancyGrid(ctx, cs, occupancyGrid, liveParams.obstacleInfluenceRadius)
    if (showVectorField) drawVectorField(ctx, cs, current, liveParams, fieldSpacing)
    // Phase 12 — LiDAR overlay goes between grid and obstacles so the scan cloud
    // is clearly visible but obstacle circles and force arrows render on top.
    const hits = lidarHitsRef.current
    if (hits.length > 0 && sensorConfigRef.current?.lidar.enabled) {
      drawLidarOverlay(ctx, cs, current.robot.position, hits, sensorConfigRef.current.lidar.range)
    }
    drawObstacles(ctx, cs, current.obstacles, liveParams)
    drawGoal(ctx, cs, current.goal)
    if (algoRef.current) drawAlgorithmOverlay(ctx, cs, algoRef.current)
    if (showComparisonTraces) drawComparisonTraces(ctx, cs, tracesRef.current)
    // When sensor mode is on, draw force vectors based on perceived obstacles so
    // the arrows show what the robot is actually responding to, not the truth.
    const perceivedForceObstacles =
      hits.length > 0 && sensorConfigRef.current?.lidar.enabled
        ? hits.filter((h) => h.hitObstacle && !h.dropped).map((h) => ({ position: h.worldPoint, radius: 0.15 }))
        : undefined
    drawForceVectors(ctx, cs, current, liveParams, perceivedForceObstacles)
    if (recoveryRef.current?.active) drawRecoveryOverlay(ctx, cs, current, recoveryRef.current, recoveryMethodRef.current, current.time)
    // Phase 15 — SLAM ghost + landmarks drawn before the true robot so the blue
    // APF robot always reads as the foreground, distinguishing true vs. estimated.
    if (showSLAM && slamStateRef.current) drawSLAMOverlay(ctx, cs, slamStateRef.current, current.robot.position, current.time)
    // Phase 14 — uncertainty ellipse behind the robot so the robot circle reads
    // clearly on top of the probabilistic region.
    if (showUncertainty && uncertaintyStats) drawUncertaintyEllipse(ctx, cs, current.robot.position, uncertaintyStats, uncertaintySigmaLevel)
    drawRobot(ctx, cs, current)
    if (localMinimaRef.current?.isTrapped) drawLocalMinimumMarker(ctx, cs, localMinimaRef.current, current.time)
    drawLegend(ctx, cs, {
      showVectorField,
      showHeatmap,
      showOccupancyGrid,
      showUncertainty,
      showSLAM,
      showComparisonTraces,
      sensorOn: !!sensorConfigRef.current?.lidar.enabled,
      algoOn: !!algoRef.current,
      recoveryOn: !!recoveryRef.current?.active,
    })
  }

  // Resizing clears the canvas, so we redraw inline rather than waiting on
  // the state-change effect below — otherwise a resize while paused leaves
  // the canvas blank until the next tick.
  useEffect(() => {
    const canvas = canvasRef.current
    const container = containerRef.current
    if (!canvas || !container) return

    const resize = () => {
      const dpr = window.devicePixelRatio || 1
      const { width, height } = container.getBoundingClientRect()
      const cs = coordsRef.current
      if (Math.abs(width - cs.width) < RESIZE_EPSILON_PX && Math.abs(height - cs.height) < RESIZE_EPSILON_PX) {
        return
      }
      canvas.width = width * dpr
      canvas.height = height * dpr
      canvas.style.width = `${width}px`
      canvas.style.height = `${height}px`
      canvas.getContext('2d')?.setTransform(dpr, 0, 0, dpr, 0, 0)
      coordsRef.current = createCoordinateSystem(width, height, pixelsPerMeter)
      draw()
    }

    resize()
    const observer = new ResizeObserver(resize)
    observer.observe(container)
    return () => observer.disconnect()
  }, [pixelsPerMeter])

  useEffect(() => {
    draw()
  }, [state, params, showVectorField, fieldSpacing, showHeatmap, heatmapResolution, comparisonTraces, localMinima, algorithmComparison, lidarHits, slamState])

  const handleClick = (event: MouseEvent<HTMLCanvasElement>) => {
    if (!onCanvasClick) return
    const rect = event.currentTarget.getBoundingClientRect()
    const screenPoint = { x: event.clientX - rect.left, y: event.clientY - rect.top }
    onCanvasClick(screenToWorld(coordsRef.current, screenPoint))
  }

  return (
    <div
      ref={containerRef}
      className="relative h-full w-full overflow-hidden border border-slate-800 bg-slate-950"
    >
      <canvas
        ref={canvasRef}
        onClick={handleClick}
        className="block h-full w-full cursor-crosshair"
      />
    </div>
  )
}

const drawGrid = (ctx: CanvasRenderingContext2D, cs: CoordinateSystem) => {
  const spacingPx = worldLengthToScreen(cs, GRID_SPACING_M)
  if (spacingPx < 8) return

  const origin = worldToScreen(cs, { x: 0, y: 0 })

  ctx.strokeStyle = 'rgba(148, 163, 184, 0.12)'
  ctx.lineWidth = 1
  for (let x = origin.x % spacingPx; x < cs.width; x += spacingPx) {
    ctx.beginPath()
    ctx.moveTo(x, 0)
    ctx.lineTo(x, cs.height)
    ctx.stroke()
  }
  for (let y = origin.y % spacingPx; y < cs.height; y += spacingPx) {
    ctx.beginPath()
    ctx.moveTo(0, y)
    ctx.lineTo(cs.width, y)
    ctx.stroke()
  }

  ctx.strokeStyle = 'rgba(148, 163, 184, 0.35)'
  ctx.beginPath()
  ctx.moveTo(origin.x, 0)
  ctx.lineTo(origin.x, cs.height)
  ctx.moveTo(0, origin.y)
  ctx.lineTo(cs.width, origin.y)
  ctx.stroke()
}

// Three-stop gradient from "calm" (low potential, near the goal) through a
// midpoint to "hot" (high potential, near obstacles or far from the goal).
// The endpoints echo colors already used elsewhere (slate background, amber
// accent) so the heatmap feels like part of the same palette, not a bolt-on.
const POTENTIAL_LOW: [number, number, number] = [15, 23, 42]
const POTENTIAL_MID: [number, number, number] = [124, 58, 237]
const POTENTIAL_HIGH: [number, number, number] = [251, 191, 36]

const lerp = (a: number, b: number, t: number) => a + (b - a) * t

const potentialColor = (t: number): [number, number, number] => {
  const clamped = Math.min(Math.max(t, 0), 1)
  const [from, to, localT] =
    clamped <= 0.5
      ? [POTENTIAL_LOW, POTENTIAL_MID, clamped / 0.5]
      : [POTENTIAL_MID, POTENTIAL_HIGH, (clamped - 0.5) / 0.5]
  return [
    Math.round(lerp(from[0], to[0], localT)),
    Math.round(lerp(from[1], to[1], localT)),
    Math.round(lerp(from[2], to[2], localT)),
  ]
}

// Potential is unbounded (it grows with distance from the goal and spikes at
// obstacle surfaces), so absolute values aren't meaningful for color — only
// relative height across what's visible is. Two passes: sample everything,
// pick a normalization range robust to outliers, then shade each cell by
// where it falls in that range (see the outlier note below for why "robust"
// matters more than it sounds like it should).
const drawPotentialHeatmap = (
  ctx: CanvasRenderingContext2D,
  cs: CoordinateSystem,
  state: SimulationState,
  params: SimulationParams,
  cellSizeM: number,
) => {
  if (!state.goal && state.obstacles.length === 0) return
  const cellPx = worldLengthToScreen(cs, cellSizeM)
  if (cellPx < 2) return

  const corner1 = screenToWorld(cs, { x: 0, y: 0 })
  const corner2 = screenToWorld(cs, { x: cs.width, y: cs.height })
  const minX = Math.min(corner1.x, corner2.x)
  const maxX = Math.max(corner1.x, corner2.x)
  const minY = Math.min(corner1.y, corner2.y)
  const maxY = Math.max(corner1.y, corner2.y)

  const startX = Math.floor(minX / cellSizeM) * cellSizeM
  const startY = Math.floor(minY / cellSizeM) * cellSizeM
  const cols = Math.floor((maxX - startX) / cellSizeM) + 1
  const rows = Math.floor((maxY - startY) / cellSizeM) + 1
  if (cols < 2 || rows < 2) return

  // Two distinct problems live in this neighborhood, and they need two
  // distinct fixes:
  //
  // (1) Physical interiors. A cell whose center falls *inside* an obstacle's
  // body has `surfaceDistance` clamped to the same 1e-3 regardless of how
  // deep inside it is — every such cell reports an identical, physically
  // meaningless value (the robot can never stand there). On a coarse grid
  // that reads as a flat, uniformly-colored square stamped on top of the
  // gradient — the exact "no gradient, just squares" artifact reported.
  // Fix: skip them outright. `drawObstacles` already paints a filled circle
  // over this exact area afterward, so nothing is lost visually — and
  // critically, these extreme/identical values stop polluting the
  // normalization range below, leaving more of it for gradient that's
  // actually visible to the robot.
  //
  // (2) Dynamic range needs TWO complementary fixes, not one:
  //
  //   a) Shape — the repulsive term grows as 1/d² approaching a surface, so
  //   the *smooth falloff zone* around one obstacle still spans several
  //   orders of magnitude on its own. Linear min/max lets the high end crush
  //   the bowl into a flat smear (the original "faded" bug); percentile
  //   clipping fixes that but saturates the whole falloff zone into one flat
  //   color (the first "square" bug). `log1p` compression fixes *both*,
  //   because log is smooth and monotonic — it preserves continuous gradient
  //   at every scale instead of cutting a band off and flattening it. This is
  //   the same move HDR tone-mapping and astronomical imaging make for
  //   "the interesting detail spans many orders of magnitude."
  //
  //   b) Outliers — log compression tames the *shape* of the falloff, but a
  //   handful of individual cells whose centers happen to land within
  //   millimeters of *some* obstacle's surface still compress to values far
  //   above everything else (their raw potential is ~10^6, vs ~0-50 for the
  //   bowl — log brings that down, but not all the way to parity). With a
  //   single obstacle, that's one or two harmless "hot pixels." With many
  //   obstacles scattered around (as the user found), the *odds* that at
  //   least one sample lands that close to *some* surface go up — and a true
  //   max anchored to that one outlier crushes everyone else's gradient into
  //   a flat band again, with the outliers themselves reading as stray
  //   colored squares. Fix: clip `hi` to the 95th percentile of the
  //   log-compressed values (not the raw ones — log has already done the
  //   heavy lifting, so this just trims the last few outlier cells rather
  //   than a whole band of them). They simply saturate at the hottest color
  //   — same tone-mapping idea, applied to the right quantity this time.
  const insideAnyObstacle = (point: Vector2) =>
    state.obstacles.some((obstacle) => distance(point, obstacle.position) < obstacle.radius)

  // Two-component normalization: attractive and repulsive are normalized
  // independently, then screen-blended so obstacle influence is always
  // visible regardless of how large the attractive bowl's range is in view.
  //
  // The attractive component spans many units (bowl grows quadratically with
  // distance from goal) so it still gets log1p + 95th-percentile treatment
  // for its own range. The repulsive component is now bounded by η/2 (the
  // quadratic cone formula's surface maximum), so a direct [0, η/2] linear
  // scale is all it needs — no log, no percentile, no gymnastics.
  //
  // Screen blend: t = 1 − (1−t_att)(1−t_rep). The attractive bowl sets the
  // base hue; the repulsive contribution "lights up" on top, reaching full
  // saturation at the obstacle surface and fading to zero at the Q* edge,
  // exactly matching the force falloff the vector field arrows already show.
  const attGrid: number[][] = []
  const repGrid: number[][] = []
  const attValues: number[] = []

  for (let xi = 0; xi < cols; xi++) {
    attGrid[xi] = []
    repGrid[xi] = []
    const x = startX + xi * cellSizeM
    for (let yi = 0; yi < rows; yi++) {
      const pos = { x, y: startY + yi * cellSizeM }
      const u_att = attractivePotentialAt(pos, state.goal, params)
      const u_rep = potentialAt(pos, state.goal, state.obstacles, params) - u_att
      attGrid[xi][yi] = Math.log1p(Math.max(u_att, 0))
      repGrid[xi][yi] = Math.max(u_rep, 0)
      if (!insideAnyObstacle(pos)) attValues.push(attGrid[xi][yi])
    }
  }
  if (attValues.length === 0) return

  attValues.sort((a, b) => a - b)
  const lo = attValues[0]
  const hi = attValues[Math.min(Math.floor(attValues.length * 0.95), attValues.length - 1)]
  const range = hi - lo || 1
  // Theoretical ceiling for the intermediate potential at the clamped surface
  // distance of 1e-3: U_max = η/Q*(ln(Q*/1e-3) − 1 + 1e-3/Q*) ≈ η/Q*·ln(Q*/1e-3).
  // Guard against zero gain/radius to avoid divide-by-zero.
  const repCeiling = Math.max(
    (params.repulsiveGain / params.obstacleInfluenceRadius) *
      Math.log(params.obstacleInfluenceRadius / 1e-3),
    1e-6,
  )

  const bitmap = document.createElement('canvas')
  bitmap.width = cols
  bitmap.height = rows
  const bctx = bitmap.getContext('2d')
  if (!bctx) return
  const image = bctx.createImageData(cols, rows)
  for (let xi = 0; xi < cols; xi++) {
    for (let yi = 0; yi < rows; yi++) {
      const t_att = Math.min(Math.max((attGrid[xi][yi] - lo) / range, 0), 1)
      const t_rep = Math.min(repGrid[xi][yi] / repCeiling, 1)
      // Screen blend: either component can push the display toward "hot"
      // independently, and together they saturate to full brightness.
      const t = 1 - (1 - t_att) * (1 - t_rep)
      const [r, g, b] = potentialColor(t)
      // World Y increases upward; bitmap rows run top-to-bottom — flip.
      const row = rows - 1 - yi
      const idx = (row * cols + xi) * 4
      image.data[idx] = r
      image.data[idx + 1] = g
      image.data[idx + 2] = b
      image.data[idx + 3] = 255
    }
  }
  bctx.putImageData(image, 0, 0)

  const topLeft = worldToScreen(cs, { x: startX - cellSizeM / 2, y: startY + (rows - 1) * cellSizeM + cellSizeM / 2 })
  const bottomRight = worldToScreen(cs, { x: startX + (cols - 1) * cellSizeM + cellSizeM / 2, y: startY - cellSizeM / 2 })

  ctx.save()
  ctx.imageSmoothingEnabled = true
  ctx.imageSmoothingQuality = 'high'
  ctx.drawImage(bitmap, topLeft.x, topLeft.y, bottomRight.x - topLeft.x, bottomRight.y - topLeft.y)
  ctx.restore()
}

// Samples the field on a grid spanning whatever's currently visible (computed
// via screenToWorld so it tracks resizes), and draws each sample as an arrow —
// direction shows where the field pushes, length + opacity show how hard.
const drawVectorField = (
  ctx: CanvasRenderingContext2D,
  cs: CoordinateSystem,
  state: SimulationState,
  params: SimulationParams,
  spacingM: number,
) => {
  if (!state.goal && state.obstacles.length === 0) return
  const spacingPx = worldLengthToScreen(cs, spacingM)
  if (spacingPx < 6) return

  const corner1 = screenToWorld(cs, { x: 0, y: 0 })
  const corner2 = screenToWorld(cs, { x: cs.width, y: cs.height })
  const minX = Math.min(corner1.x, corner2.x)
  const maxX = Math.max(corner1.x, corner2.x)
  const minY = Math.min(corner1.y, corner2.y)
  const maxY = Math.max(corner1.y, corner2.y)

  const maxLengthPx = spacingPx * FIELD_ARROW_CELL_RATIO
  const startX = Math.floor(minX / spacingM) * spacingM
  const startY = Math.floor(minY / spacingM) * spacingM

  for (let x = startX; x <= maxX; x += spacingM) {
    for (let y = startY; y <= maxY; y += spacingM) {
      const sample = { x, y }
      const force = fieldForceAt(sample, state.goal, state.obstacles, params)
      const mag = magnitude(force)
      if (mag < 1e-4) continue

      const lengthPx = Math.min(mag * FIELD_ARROW_SCALE, maxLengthPx)
      const dir = normalize(force)
      const from = worldToScreen(cs, sample)
      const to = { x: from.x + dir.x * lengthPx, y: from.y - dir.y * lengthPx }
      const alpha = 0.12 + 0.7 * Math.min(lengthPx / maxLengthPx, 1)
      drawArrow(ctx, from, to, {
        color: `rgba(94, 234, 212, ${alpha.toFixed(2)})`,
        lineWidth: 1.25,
        headLength: Math.min(5, lengthPx * 0.5),
      })
    }
  }
}

const drawObstacles = (
  ctx: CanvasRenderingContext2D,
  cs: CoordinateSystem,
  obstacles: Obstacle[],
  params: SimulationParams,
) => {
  for (const obstacle of obstacles) {
    const center = worldToScreen(cs, obstacle.position)
    const radiusPx = worldLengthToScreen(cs, obstacle.radius)
    const influencePx = worldLengthToScreen(cs, obstacle.radius + params.obstacleInfluenceRadius)

    // Dashed ring at radius + Q* marks where the repulsive field kicks in.
    // A faint translucent line reads fine against the plain dark background,
    // but the heatmap (Phase 6) paints that whole area in similarly warm
    // amber/violet tones — low contrast made the ring nearly disappear right
    // where it matters most. Fix: a dark halo stroke underneath, the same
    // trick topographic contour lines use to stay legible over any terrain
    // color — it guarantees contrast whether the cell beneath is bright
    // amber or deep violet, without needing to know which.
    ctx.save()
    ctx.setLineDash([5, 5])
    ctx.strokeStyle = 'rgba(15, 23, 42, 0.7)'
    ctx.lineWidth = 3
    ctx.beginPath()
    ctx.arc(center.x, center.y, influencePx, 0, Math.PI * 2)
    ctx.stroke()
    ctx.strokeStyle = 'rgba(248, 113, 113, 0.85)'
    ctx.lineWidth = 1.5
    ctx.beginPath()
    ctx.arc(center.x, center.y, influencePx, 0, Math.PI * 2)
    ctx.stroke()
    ctx.restore()

    ctx.fillStyle = 'rgba(248, 113, 113, 0.85)'
    ctx.beginPath()
    ctx.arc(center.x, center.y, radiusPx, 0, Math.PI * 2)
    ctx.fill()
    ctx.strokeStyle = '#ef4444'
    ctx.lineWidth = 2
    ctx.stroke()
  }
}

const drawGoal = (ctx: CanvasRenderingContext2D, cs: CoordinateSystem, goal: Vector2 | null) => {
  if (!goal) return
  const center = worldToScreen(cs, goal)
  const radiusPx = worldLengthToScreen(cs, GOAL_RADIUS_M)

  ctx.strokeStyle = '#4ade80'
  ctx.lineWidth = 2
  ctx.beginPath()
  ctx.arc(center.x, center.y, radiusPx, 0, Math.PI * 2)
  ctx.stroke()

  // crosshair so the exact target point reads clearly at any zoom
  ctx.beginPath()
  ctx.moveTo(center.x - radiusPx * 1.6, center.y)
  ctx.lineTo(center.x + radiusPx * 1.6, center.y)
  ctx.moveTo(center.x, center.y - radiusPx * 1.6)
  ctx.lineTo(center.x, center.y + radiusPx * 1.6)
  ctx.stroke()
}

const drawForceVector = (
  ctx: CanvasRenderingContext2D,
  cs: CoordinateSystem,
  origin: Vector2,
  force: Vector2,
  color: string,
) => {
  const mag = magnitude(force)
  if (mag < 1e-6) return

  const lengthPx = Math.min(mag * FORCE_ARROW_SCALE, MAX_FORCE_ARROW_PX)
  const dir = normalize(force)
  const from = worldToScreen(cs, origin)
  const to = { x: from.x + dir.x * lengthPx, y: from.y - dir.y * lengthPx }
  drawArrow(ctx, from, to, { color, lineWidth: 2, headLength: 9 })
}

// Attractive, repulsive, and their sum (the resultant that actually drives
// the robot) each get their own arrow so the balance between them is visible.
const drawForceVectors = (
  ctx: CanvasRenderingContext2D,
  cs: CoordinateSystem,
  state: SimulationState,
  params: SimulationParams,
  // When provided, these replace state.obstacles for force computation — used
  // in sensor mode so the arrows show perceived forces, not ground-truth forces.
  forceObstacles?: Obstacle[],
) => {
  const obstacles = forceObstacles ?? state.obstacles
  if (!state.goal && obstacles.length === 0) return
  const { attractive, repulsive, net } = computeForces(state.robot, state.goal, obstacles, params)
  drawForceVector(ctx, cs, state.robot.position, attractive, '#facc15')
  drawForceVector(ctx, cs, state.robot.position, repulsive, '#f87171')
  drawForceVector(ctx, cs, state.robot.position, net, '#e2e8f0')
}

// Each ghost gets a fading polyline for where it's been plus a small dot for
// where it is now — same idea as the main robot's heading line, just dimmer
// so the active trajectory still reads as the "real" one.
const drawComparisonTraces = (ctx: CanvasRenderingContext2D, cs: CoordinateSystem, traces: ComparisonTrace[]) => {
  for (const trace of traces) {
    const [r, g, b] = TRACE_COLORS[trace.method]

    if (trace.trail.length > 1) {
      ctx.strokeStyle = `rgba(${r}, ${g}, ${b}, 0.3)`
      ctx.lineWidth = 1.5
      ctx.beginPath()
      trace.trail.forEach((point, i) => {
        const screen = worldToScreen(cs, point)
        if (i === 0) ctx.moveTo(screen.x, screen.y)
        else ctx.lineTo(screen.x, screen.y)
      })
      ctx.stroke()
    }

    const center = worldToScreen(cs, trace.robot.position)
    const radiusPx = worldLengthToScreen(cs, GHOST_RADIUS_M)
    ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 0.6)`
    ctx.beginPath()
    ctx.arc(center.x, center.y, radiusPx, 0, Math.PI * 2)
    ctx.fill()
  }
}

const drawRobot = (ctx: CanvasRenderingContext2D, cs: CoordinateSystem, state: SimulationState) => {
  const { position, velocity } = state.robot
  const center = worldToScreen(cs, position)
  const radiusPx = worldLengthToScreen(cs, ROBOT_RADIUS_M)

  ctx.fillStyle = '#38bdf8'
  ctx.beginPath()
  ctx.arc(center.x, center.y, radiusPx, 0, Math.PI * 2)
  ctx.fill()
  ctx.strokeStyle = '#0ea5e9'
  ctx.lineWidth = 2
  ctx.stroke()

  // short heading line in the direction of travel
  if (magnitude(velocity) > 1e-6) {
    const dir = normalize(velocity)
    ctx.strokeStyle = '#e2e8f0'
    ctx.lineWidth = 2
    ctx.beginPath()
    ctx.moveTo(center.x, center.y)
    ctx.lineTo(center.x + dir.x * radiusPx * 1.8, center.y - dir.y * radiusPx * 1.8)
    ctx.stroke()
  }
}

// Phase 9 — visualizes whichever recovery strategy is active:
//   waypoint     → diamond marker at the temp sub-goal + dashed line from robot
//   virtual-force → emerald arrow showing the perpendicular push direction
//   random-walk  → pulsing dashed ring around the robot (force direction changes
//                  every 0.4s sim-time, so showing a snapshot arrow would be
//                  misleading; a ring reads "stochastic push" more honestly)
const drawRecoveryOverlay = (
  ctx: CanvasRenderingContext2D,
  cs: CoordinateSystem,
  state: SimulationState,
  recovery: RecoveryInfo,
  recoveryMethod: RecoveryMethod,
  time: number,
) => {
  const robotScreen = worldToScreen(cs, state.robot.position)

  if (recovery.tempWaypoint) {
    const wpScreen = worldToScreen(cs, recovery.tempWaypoint)
    ctx.save()
    ctx.strokeStyle = 'rgba(52, 211, 153, 0.5)'
    ctx.lineWidth = 1.5
    ctx.setLineDash([5, 5])
    ctx.beginPath()
    ctx.moveTo(robotScreen.x, robotScreen.y)
    ctx.lineTo(wpScreen.x, wpScreen.y)
    ctx.stroke()
    ctx.setLineDash([])
    const r = 8
    ctx.strokeStyle = '#34d399'
    ctx.lineWidth = 2
    ctx.fillStyle = 'rgba(52, 211, 153, 0.2)'
    ctx.beginPath()
    ctx.moveTo(wpScreen.x, wpScreen.y - r)
    ctx.lineTo(wpScreen.x + r, wpScreen.y)
    ctx.lineTo(wpScreen.x, wpScreen.y + r)
    ctx.lineTo(wpScreen.x - r, wpScreen.y)
    ctx.closePath()
    ctx.fill()
    ctx.stroke()
    ctx.restore()
  } else if (recoveryMethod === 'virtual-force' && state.goal && state.obstacles.length > 0) {
    const toGoal = normalize(sub(state.goal, state.robot.position))
    const perp1: Vector2 = { x: -toGoal.y, y: toGoal.x }
    const perp2: Vector2 = { x: toGoal.y, y: -toGoal.x }
    const nearest = state.obstacles.reduce(
      (best, o) => (distance(state.robot.position, o.position) < distance(state.robot.position, best.position) ? o : best),
      state.obstacles[0],
    )
    const toNearest = sub(nearest.position, state.robot.position)
    const chosen = dot(perp1, toNearest) <= 0 ? perp1 : perp2
    drawForceVector(ctx, cs, state.robot.position, scale(chosen, 5), '#34d399')
  } else if (recoveryMethod === 'random-walk') {
    const pulse = (Math.sin(time * 6) + 1) / 2
    const r = worldLengthToScreen(cs, ROBOT_RADIUS_M * 2.2) + pulse * 4
    ctx.save()
    ctx.strokeStyle = `rgba(52, 211, 153, ${0.3 + pulse * 0.3})`
    ctx.lineWidth = 1.5
    ctx.setLineDash([3, 4])
    ctx.beginPath()
    ctx.arc(robotScreen.x, robotScreen.y, r, 0, Math.PI * 2)
    ctx.stroke()
    ctx.setLineDash([])
    ctx.restore()
  }
}

const TRAP_LABEL = 'LOCAL MINIMUM'

// Phase 8 — labels the exact world point where ∇U went flat away from the
// goal: a critical point of the potential that isn't the destination. The
// ring pulses (driven by sim time, so it stays in sync whether running or
// stepped) to read as "alive" the way the canvas's other live readouts do,
// and the halo-stroked label borrows the same contour-map trick `drawObstacles`
// uses to stay legible over the heatmap, the grid, or open space alike.
const drawLocalMinimumMarker = (
  ctx: CanvasRenderingContext2D,
  cs: CoordinateSystem,
  diagnostics: LocalMinimaDiagnostics,
  time: number,
) => {
  if (!diagnostics.trapPosition) return
  const center = worldToScreen(cs, diagnostics.trapPosition)
  const pulse = (Math.sin(time * 4) + 1) / 2 // 0..1, ~0.8s period
  const baseRadiusPx = worldLengthToScreen(cs, ROBOT_RADIUS_M * 1.6)
  const ringRadiusPx = baseRadiusPx + pulse * 8

  ctx.save()
  ctx.strokeStyle = `rgba(251, 113, 133, ${0.85 - pulse * 0.4})`
  ctx.lineWidth = 2
  ctx.beginPath()
  ctx.arc(center.x, center.y, ringRadiusPx, 0, Math.PI * 2)
  ctx.stroke()

  // crosshair marking the precise critical point — same language as drawGoal,
  // in rose rather than green, so the two read as "the same kind of thing,
  // different verdicts" (arrived vs. trapped).
  ctx.strokeStyle = 'rgba(251, 113, 133, 0.9)'
  ctx.lineWidth = 1.5
  ctx.beginPath()
  ctx.moveTo(center.x - baseRadiusPx, center.y)
  ctx.lineTo(center.x + baseRadiusPx, center.y)
  ctx.moveTo(center.x, center.y - baseRadiusPx)
  ctx.lineTo(center.x, center.y + baseRadiusPx)
  ctx.stroke()

  ctx.font = '11px ui-monospace, monospace'
  ctx.textAlign = 'center'
  ctx.textBaseline = 'bottom'
  const labelY = center.y - ringRadiusPx - 6
  ctx.lineWidth = 3
  ctx.strokeStyle = 'rgba(15, 23, 42, 0.85)'
  ctx.strokeText(TRAP_LABEL, center.x, labelY)
  ctx.fillStyle = 'rgba(251, 113, 133, 0.95)'
  ctx.fillText(TRAP_LABEL, center.x, labelY)
  ctx.restore()
}

// ─── Phase 11: Algorithm Comparison Overlay ───────────────────────────────────
// A* path  = cyan (#06b6d4), robot = filled cyan circle
// RRT tree = faint slate lines, path = orange (#f97316), robot = filled orange circle
// Drawn before the APF robot so APF always reads as the "active" one on top.

// A* = cyan-400, RRT = amber-400 (distinct from the Euler ghost which uses orange-400)
const ASTAR_COLOR = '#22d3ee'   // cyan-400
const RRT_PATH_COLOR = '#fbbf24' // amber-400

const drawFollowerTrail = (
  ctx: CanvasRenderingContext2D,
  cs: CoordinateSystem,
  trail: Vector2[],
  color: string,
) => {
  if (trail.length < 2) return
  const len = trail.length
  ctx.save()
  for (let i = 1; i < len; i++) {
    const alpha = 0.12 + 0.55 * (i / len)
    ctx.strokeStyle = color.replace(')', `, ${alpha})`).replace('rgb(', 'rgba(').replace('#', '')
    // Hex to rgba is annoying — just use globalAlpha instead
    ctx.globalAlpha = alpha
    ctx.strokeStyle = color
    ctx.lineWidth = 1.5
    ctx.beginPath()
    const a = worldToScreen(cs, trail[i - 1])
    const b = worldToScreen(cs, trail[i])
    ctx.moveTo(a.x, a.y)
    ctx.lineTo(b.x, b.y)
    ctx.stroke()
  }
  ctx.globalAlpha = 1
  ctx.restore()
}

const drawPathPolyline = (
  ctx: CanvasRenderingContext2D,
  cs: CoordinateSystem,
  path: Vector2[],
  color: string,
  dashLen = 5,
) => {
  if (path.length < 2) return
  ctx.save()
  ctx.strokeStyle = color
  ctx.lineWidth = 1.5
  ctx.globalAlpha = 0.7
  ctx.setLineDash([dashLen, 3])
  ctx.beginPath()
  const p0 = worldToScreen(cs, path[0])
  ctx.moveTo(p0.x, p0.y)
  for (let i = 1; i < path.length; i++) {
    const p = worldToScreen(cs, path[i])
    ctx.lineTo(p.x, p.y)
  }
  ctx.stroke()
  ctx.setLineDash([])
  ctx.globalAlpha = 1
  ctx.restore()
}

// Follower robots are hollow rings so they're instantly distinct from the APF
// robot (solid fill) even at a glance. A label above each ring removes any
// remaining ambiguity with the integration comparison ghosts.
const drawFollowerRobot = (
  ctx: CanvasRenderingContext2D,
  cs: CoordinateSystem,
  pos: Vector2,
  vel: Vector2,
  color: string,
  label: string,
) => {
  const center = worldToScreen(cs, pos)
  const radiusPx = worldLengthToScreen(cs, ROBOT_RADIUS_M * 0.9)
  ctx.save()

  // Hollow ring
  ctx.strokeStyle = color
  ctx.lineWidth = 2.5
  ctx.globalAlpha = 0.95
  ctx.beginPath()
  ctx.arc(center.x, center.y, radiusPx, 0, Math.PI * 2)
  ctx.stroke()

  // Small center dot so the robot is visible when heading is ambiguous
  ctx.fillStyle = color
  ctx.globalAlpha = 0.7
  ctx.beginPath()
  ctx.arc(center.x, center.y, radiusPx * 0.22, 0, Math.PI * 2)
  ctx.fill()

  // Heading line
  const speed = Math.sqrt(vel.x * vel.x + vel.y * vel.y)
  if (speed > 0.05) {
    const hdg = Math.atan2(-vel.y, vel.x)
    ctx.globalAlpha = 0.95
    ctx.strokeStyle = color
    ctx.lineWidth = 1.5
    ctx.beginPath()
    ctx.moveTo(center.x, center.y)
    ctx.lineTo(center.x + Math.cos(hdg) * radiusPx * 1.45, center.y - Math.sin(hdg) * radiusPx * 1.45)
    ctx.stroke()
  }

  // Label above the ring — halo stroke ensures legibility over any background
  const labelY = center.y - radiusPx - 5
  ctx.font = 'bold 9px ui-monospace, monospace'
  ctx.textAlign = 'center'
  ctx.textBaseline = 'bottom'
  ctx.globalAlpha = 1
  ctx.lineWidth = 2.5
  ctx.strokeStyle = 'rgba(11,15,23,0.85)'
  ctx.strokeText(label, center.x, labelY)
  ctx.fillStyle = color
  ctx.fillText(label, center.x, labelY)

  ctx.globalAlpha = 1
  ctx.restore()
}

/**
 * Draws the LiDAR scan: a faint range ring, ray lines (lime for hits, dim for
 * free-space), hit dots at the measured surface position, and small amber dots
 * at the true surface position so the range error is directly visible.
 */
const drawLidarOverlay = (
  ctx: CanvasRenderingContext2D,
  cs: CoordinateSystem,
  robotPos: Vector2,
  hits: LidarHit[],
  maxRange: number,
) => {
  ctx.save()

  const robotPx = worldToScreen(cs, robotPos)
  const rangePx = worldLengthToScreen(cs, maxRange)

  // Faint dashed range ring
  ctx.beginPath()
  ctx.arc(robotPx.x, robotPx.y, rangePx, 0, Math.PI * 2)
  ctx.strokeStyle = 'rgba(100,116,139,0.14)'
  ctx.lineWidth = 1
  ctx.setLineDash([4, 7])
  ctx.stroke()
  ctx.setLineDash([])

  for (const hit of hits) {
    if (hit.dropped) continue

    const endPx = worldToScreen(cs, hit.worldPoint)

    // Ray line — bright lime for obstacle hits, dim slate for free-space rays
    ctx.beginPath()
    ctx.moveTo(robotPx.x, robotPx.y)
    ctx.lineTo(endPx.x, endPx.y)
    ctx.strokeStyle = hit.hitObstacle ? 'rgba(163,230,53,0.75)' : 'rgba(100,116,139,0.20)'
    ctx.lineWidth = hit.hitObstacle ? 1.2 : 0.7
    ctx.stroke()

    if (!hit.hitObstacle) continue

    // Lime dot at the *measured* (noisy) surface position
    ctx.beginPath()
    ctx.arc(endPx.x, endPx.y, 3.5, 0, Math.PI * 2)
    ctx.fillStyle = 'rgba(163,230,53,1.0)'
    ctx.fill()

    // Amber dot at the *true* surface position — visually shows the error gap
    if (Math.abs(hit.measuredRange - hit.trueRange) > 0.03) {
      const dir: Vector2 = { x: Math.cos(hit.angle), y: Math.sin(hit.angle) }
      const truePt: Vector2 = {
        x: robotPos.x + hit.trueRange * dir.x,
        y: robotPos.y + hit.trueRange * dir.y,
      }
      const truePx = worldToScreen(cs, truePt)
      ctx.beginPath()
      ctx.arc(truePx.x, truePx.y, 2.5, 0, Math.PI * 2)
      ctx.fillStyle = 'rgba(251,191,36,0.85)'
      ctx.fill()
    }
  }

  ctx.restore()
}

// ─── Canvas legend ────────────────────────────────────────────────────────────
// Drawn last so it's always on top. Shows permanent items (robot, goal, obstacle,
// forces) plus conditional rows for active overlays.

interface LegendFlags {
  showVectorField: boolean
  showHeatmap: boolean
  showOccupancyGrid: boolean
  showUncertainty: boolean
  showSLAM: boolean
  showComparisonTraces: boolean
  sensorOn: boolean
  algoOn: boolean
  recoveryOn: boolean
}

const drawLegend = (
  ctx: CanvasRenderingContext2D,
  cs: CoordinateSystem,
  flags: LegendFlags,
) => {
  const ROW_H = 18
  const PAD_X = 10
  const PAD_Y = 8
  const SWATCH_W = 12
  const SWATCH_H = 10
  const FONT = '11px ui-monospace, monospace'

  const permanentItems: Array<{ color: string; label: string }> = [
    { color: '#60a5fa', label: 'Robot (APF)' },
    { color: '#4ade80', label: 'Goal' },
    { color: '#f87171', label: 'Obstacle' },
    { color: '#facc15', label: 'Attractive force' },
    { color: '#fb923c', label: 'Repulsive force' },
    { color: '#f8fafc', label: 'Net force' },
  ]

  const conditionalItems: Array<{ color: string; label: string }> = []
  if (flags.showVectorField) conditionalItems.push({ color: '#2dd4bf', label: 'Vector field' })
  if (flags.showHeatmap) conditionalItems.push({ color: '#f59e0b', label: 'Potential heatmap' })
  if (flags.showOccupancyGrid) conditionalItems.push({ color: '#38bdf8', label: 'Occupancy grid' })
  if (flags.showUncertainty) conditionalItems.push({ color: '#818cf8', label: 'Uncertainty ellipse' })
  if (flags.showSLAM) conditionalItems.push({ color: '#fde047', label: 'SLAM ghost / landmarks' })
  if (flags.showComparisonTraces) conditionalItems.push({ color: '#fb923c', label: 'Euler ghost' },
    { color: '#e879f9', label: 'SIE ghost' }, { color: '#34d399', label: 'RK4 ghost' })
  if (flags.sensorOn) conditionalItems.push({ color: '#a3e635', label: 'LiDAR scan' })
  if (flags.algoOn) {
    conditionalItems.push({ color: '#06b6d4', label: 'A* path' })
    conditionalItems.push({ color: '#f97316', label: 'RRT path' })
  }
  if (flags.recoveryOn) conditionalItems.push({ color: '#34d399', label: 'Recovery' })

  const items = [...permanentItems, ...(conditionalItems.length > 0 ? [{ color: '', label: '──────────' }, ...conditionalItems] : [])]

  ctx.save()
  ctx.font = FONT
  const maxLabelW = items.reduce((m, it) => Math.max(m, ctx.measureText(it.label).width), 0)
  const boxW = PAD_X * 2 + SWATCH_W + 6 + maxLabelW
  const boxH = PAD_Y * 2 + items.length * ROW_H

  const x = 12
  const y = cs.height - boxH - 12

  // Dark panel background
  ctx.fillStyle = 'rgba(11,15,23,0.88)'
  ctx.fillRect(x, y, boxW, boxH)
  ctx.strokeStyle = 'rgba(51,65,85,0.9)'
  ctx.lineWidth = 1
  ctx.strokeRect(x + 0.5, y + 0.5, boxW - 1, boxH - 1)

  for (let i = 0; i < items.length; i++) {
    const item = items[i]
    const rowY = y + PAD_Y + i * ROW_H

    if (!item.color) {
      // Separator row
      ctx.fillStyle = 'rgba(71,85,105,0.6)'
      ctx.fillText(item.label, x + PAD_X, rowY + SWATCH_H - 1)
      continue
    }

    // Color swatch
    ctx.fillStyle = item.color
    ctx.fillRect(x + PAD_X, rowY, SWATCH_W, SWATCH_H)

    // Label
    ctx.fillStyle = '#cbd5e1'
    ctx.fillText(item.label, x + PAD_X + SWATCH_W + 6, rowY + SWATCH_H - 1)
  }
  ctx.restore()
}

const drawAlgorithmOverlay = (
  ctx: CanvasRenderingContext2D,
  cs: CoordinateSystem,
  algo: AlgorithmComparisonState,
) => {
  // ── RRT: path, then robot ────────────────────────────────────────────────
  // Exploration tree lines removed — too visually noisy for normal use.
  if (algo.rrt.plan) drawPathPolyline(ctx, cs, algo.rrt.plan.path, RRT_PATH_COLOR)
  if (algo.rrt.follower) {
    drawFollowerTrail(ctx, cs, algo.rrt.follower.trail, RRT_PATH_COLOR)
    drawFollowerRobot(ctx, cs, algo.rrt.follower.robot.position, algo.rrt.follower.robot.velocity, RRT_PATH_COLOR, 'RRT')
  }

  // ── A*: path, then robot (drawn on top of RRT) ───────────────────────────
  if (algo.astar.plan) drawPathPolyline(ctx, cs, algo.astar.plan.path, ASTAR_COLOR)
  if (algo.astar.follower) {
    drawFollowerTrail(ctx, cs, algo.astar.follower.trail, ASTAR_COLOR)
    drawFollowerRobot(ctx, cs, algo.astar.follower.robot.position, algo.astar.follower.robot.velocity, ASTAR_COLOR, 'A*')
  }
}
