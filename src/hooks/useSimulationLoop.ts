import { useCallback, useEffect, useRef, useState } from 'react'
import { add, distance, dot, limit, magnitude, normalize, scale, sub, zero, type Vector2 } from '@/lib/vector'
import {
  createInitialState,
  integrateRobot,
  type IntegrationMethod,
  type RobotState,
  type SimulationState,
} from '@/sim/simulationEngine'
import { attractiveForce, DEFAULT_PARAMS, fieldForceAt, type Obstacle, type SimulationParams } from '@/sim/forces'
import { DEFAULT_SENSOR_CONFIG, hitsToPerceivedObstacles, simulateLidar, type LidarHit, type SensorConfig } from '@/sim/sensors'
import { clearGrid, createGrid, DEFAULT_GRID_CONFIG, gridStats, updateGridFromHits, type OccupancyGrid, type OccupancyGridStats } from '@/sim/occupancyGrid'
import { correctCovariance, covToStats, DEFAULT_UNCERTAINTY_CONFIG, INITIAL_COVARIANCE, propagateCovariance, type Covariance2, type UncertaintyConfig, type UncertaintyStats } from '@/sim/uncertainty'
import { createInitialSLAMState, DEFAULT_SLAM_CONFIG, updateSLAM, type SLAMConfig, type SLAMState } from '@/sim/slam'
import {
  exportCSV,
  exportJSON,
  finalizeRun,
  getSavedConfigs,
  liveSnapshot,
  persistConfig,
  removeConfig,
  seedMetricsTracker,
  type ExperimentConfig,
  type LiveMetrics,
  type MetricsTracker,
  type RunMetrics,
} from '@/sim/metrics'
import { planAStar } from '@/sim/astar'
import { planRRT, type RRTResult } from '@/sim/rrt'
import { computeMinClearance, computePathLength, computeSmoothness } from '@/sim/plannerUtils'

// A stalled tab can pile up a huge accumulator; cap how many steps we'll burn
// through in one frame so it eases back in instead of freezing the page.
const MAX_SUBSTEPS_PER_FRAME = 10
const OBSTACLE_RADIUS = 0.5

const ALL_METHODS: IntegrationMethod[] = ['euler', 'semi-implicit-euler', 'rk4']

// How many recent positions each comparison ghost drags behind it. At 60Hz
// that's a few seconds of trail — enough to see a method spiral or settle
// without turning the canvas into spaghetti.
const TRAIL_LENGTH = 240

const seedTraces = (robot: RobotState): Record<IntegrationMethod, RobotState> => ({
  euler: { ...robot },
  'semi-implicit-euler': { ...robot },
  rk4: { ...robot },
})

const seedTrails = (): Record<IntegrationMethod, Vector2[]> => ({
  euler: [],
  'semi-implicit-euler': [],
  rk4: [],
})

export interface ComparisonTrace {
  method: IntegrationMethod
  robot: RobotState
  trail: Vector2[]
}

// Phase 8 — local minima are the classic potential-field failure: the
// attractive pull toward the goal and the repulsive push from obstacle(s)
// can cancel at some point that *isn't* the goal, so the net conservative
// force (the gradient of the potential, ∇U) vanishes and the robot simply
// stops — a critical point that's a trap rather than the destination.
//
// Telling "trapped" apart from "arrived" comes down to two questions: is the
// gradient (near) zero, and are we actually near the goal? Both are critical
// points of U, but only one of them is the global minimum.
const CRITICAL_GRADIENT_EPSILON = 0.4 // N (mass = 1) — net field force this small reads as "flat ground"
const GOAL_ARRIVAL_RADIUS = 1 // m — inside this ring, a vanishing gradient means "arrived", not "trapped"
const TRAP_HOLD_SECONDS = 1.5 // how long the gradient must stay flat before calling it a trap, not a pause

export interface LocalMinimaDiagnostics {
  gradientMagnitude: number
  distanceToGoal: number | null
  nearCriticalPoint: boolean
  stuckFor: number
  isTrapped: boolean
  trapPosition: Vector2 | null
}

const initialDiagnostics = (): LocalMinimaDiagnostics => ({
  gradientMagnitude: 0,
  distanceToGoal: null,
  nearCriticalPoint: false,
  stuckFor: 0,
  isTrapped: false,
  trapPosition: null,
})

// Mutable tracker carried between ticks — `lowSince` is the sim time the
// gradient first went flat away from the goal (cleared the moment it picks
// back up), and `trapPosition` freezes *where* that happened so the label
// doesn't drift once the robot's residual velocity nudges it around.
interface TrapTracker {
  lowSince: number | null
  trapPosition: Vector2 | null
}

const seedTrapTracker = (): TrapTracker => ({ lowSince: null, trapPosition: null })

const evaluateLocalMinima = (
  tracker: TrapTracker,
  time: number,
  robot: RobotState,
  goal: Vector2 | null,
  obstacles: Obstacle[],
  params: SimulationParams,
): LocalMinimaDiagnostics => {
  const gradientMagnitude = magnitude(fieldForceAt(robot.position, goal, obstacles, params))
  const distanceToGoal = goal ? distance(robot.position, goal) : null
  const nearCriticalPoint = gradientMagnitude < CRITICAL_GRADIENT_EPSILON
  const arrived = distanceToGoal !== null && distanceToGoal < GOAL_ARRIVAL_RADIUS

  if (nearCriticalPoint && !arrived) {
    if (tracker.lowSince === null) {
      tracker.lowSince = time
      tracker.trapPosition = robot.position
    }
  } else {
    tracker.lowSince = null
    tracker.trapPosition = null
  }

  const stuckFor = tracker.lowSince === null ? 0 : time - tracker.lowSince
  return {
    gradientMagnitude,
    distanceToGoal,
    nearCriticalPoint,
    stuckFor,
    isTrapped: stuckFor >= TRAP_HOLD_SECONDS,
    trapPosition: tracker.trapPosition,
  }
}

// ─── Phase 9: Recovery Methods ────────────────────────────────────────────────
//
// When the robot is stuck at a local minimum, three strategies can escape it.
// Each injects an extra force on top of the normal field forces every tick:
//
//   random-walk  — a random direction held for ~0.4s sim-time, then re-rolled.
//                  Stochastic; will escape any trap eventually, but path is
//                  chaotic and wasteful.
//   virtual-force — a push perpendicular to the current attractive direction
//                  (i.e., "around" the obstacle rather than through it).
//                  Deterministic; exploits the saddle-point geometry of most
//                  traps. Fails if the perpendicular leads into another obstacle.
//   waypoint      — places a temporary goal offset perpendicular to the
//                  attractive axis, giving the robot a reachable sub-goal that
//                  moves it out of the trap region before resuming.
//                  Most path-efficient; trades stochasticity for pre-planning.

export type RecoveryMethod = 'none' | 'random-walk' | 'virtual-force' | 'waypoint'

export interface RecoveryMethodStats {
  escapeCount: number
  totalEscapeSeconds: number
}

export interface RecoveryInfo {
  active: boolean
  tempWaypoint: Vector2 | null
  stats: Record<Exclude<RecoveryMethod, 'none'>, RecoveryMethodStats>
}

const RANDOM_WALK_MAGNITUDE = 4 // N — strong enough to break free of typical traps
const RANDOM_WALK_PERIOD = 0.4 // sim-seconds per random direction
const VIRTUAL_FORCE_MAGNITUDE = 5 // N
const WAYPOINT_GAIN = 3 // attractive gain toward the temp waypoint
const WAYPOINT_DISTANCE = 2.5 // m — how far off-axis the waypoint is placed
const WAYPOINT_ARRIVAL_RADIUS = 0.7 // m — close enough = "reached waypoint"
// Robot must move this far from the recorded trap position before an escape is
// counted. Without this, the recovery force itself can briefly lift the gradient
// above CRITICAL_GRADIENT_EPSILON on the very next tick, which the old code
// interpreted as an immediate escape (false positive).
const ESCAPE_DISTANCE = 1.5 // m

const initialRecoveryStats = (): Record<Exclude<RecoveryMethod, 'none'>, RecoveryMethodStats> => ({
  'random-walk': { escapeCount: 0, totalEscapeSeconds: 0 },
  'virtual-force': { escapeCount: 0, totalEscapeSeconds: 0 },
  waypoint: { escapeCount: 0, totalEscapeSeconds: 0 },
})

// Per-tick mutable tracker kept in a ref so the rAF loop reads current values
// without stale closure problems.
interface RecoveryTracker {
  wasTrapped: boolean
  trappedSince: number | null // sim time when robot first became trapped
  escapingFrom: Vector2 | null // world position where trap was first confirmed — escape
  // is only declared when the robot has moved ESCAPE_DISTANCE from this point,
  // so a brief gradient spike from the recovery force itself doesn't count.
  rwPhase: number // which 0.4s window we're in for random-walk direction
  rwDirection: Vector2
  tempWaypoint: Vector2 | null
  stats: Record<Exclude<RecoveryMethod, 'none'>, RecoveryMethodStats>
}

const seedRecoveryTracker = (): RecoveryTracker => ({
  wasTrapped: false,
  trappedSince: null,
  escapingFrom: null,
  rwPhase: -1,
  rwDirection: { x: 1, y: 0 },
  tempWaypoint: null,
  stats: initialRecoveryStats(),
})

// Pick a waypoint perpendicular to the trap→goal axis, far enough to be
// outside the obstacle's influence zone. Tries the left-hand side first,
// then the right if the left lands inside an obstacle.
const placeWaypoint = (trapPos: Vector2, goal: Vector2, obstacles: Obstacle[], params: SimulationParams): Vector2 => {
  const toGoal = normalize(sub(goal, trapPos))
  const perps: Vector2[] = [
    { x: -toGoal.y, y: toGoal.x },
    { x: toGoal.y, y: -toGoal.x },
  ]
  for (const p of perps) {
    const candidate = add(trapPos, scale(p, WAYPOINT_DISTANCE))
    const inside = obstacles.some(
      (o) => distance(candidate, o.position) < o.radius + params.obstacleInfluenceRadius * 0.5,
    )
    if (!inside) return candidate
  }
  // Fallback: just use the first perpendicular regardless
  return add(trapPos, scale(perps[0], WAYPOINT_DISTANCE))
}

// Compute the extra force to inject this tick for the active recovery method.
const recoveryForceFor = (
  method: RecoveryMethod,
  tracker: RecoveryTracker,
  robot: RobotState,
  goal: Vector2 | null,
  obstacles: Obstacle[],
  params: SimulationParams,
  simTime: number,
): Vector2 => {
  if (method === 'none') return zero()

  if (method === 'random-walk') {
    const phase = Math.floor(simTime / RANDOM_WALK_PERIOD)
    if (phase !== tracker.rwPhase) {
      tracker.rwPhase = phase
      const angle = Math.random() * 2 * Math.PI
      tracker.rwDirection = { x: Math.cos(angle), y: Math.sin(angle) }
    }
    return scale(tracker.rwDirection, RANDOM_WALK_MAGNITUDE)
  }

  if (method === 'virtual-force') {
    if (!goal) return zero()
    // Push perpendicular to the attractive axis — "around" the trap rather than
    // through it. Pick the side that points away from the nearest obstacle.
    const toGoal = normalize(sub(goal, robot.position))
    const perp1: Vector2 = { x: -toGoal.y, y: toGoal.x }
    const perp2: Vector2 = { x: toGoal.y, y: -toGoal.x }
    const nearest = obstacles.reduce(
      (best, o) => (distance(robot.position, o.position) < distance(robot.position, best.position) ? o : best),
      obstacles[0],
    )
    if (!nearest) return scale(perp1, VIRTUAL_FORCE_MAGNITUDE)
    const toNearest = sub(nearest.position, robot.position)
    // Choose the perp that moves AWAY from the nearest obstacle
    const chosen = dot(perp1, toNearest) <= 0 ? perp1 : perp2
    return scale(chosen, VIRTUAL_FORCE_MAGNITUDE)
  }

  if (method === 'waypoint') {
    if (!tracker.tempWaypoint) return zero()
    // Check arrival — if we've reached the waypoint, clear it so the robot
    // returns to chasing the real goal on its own next tick.
    if (distance(robot.position, tracker.tempWaypoint) < WAYPOINT_ARRIVAL_RADIUS) {
      tracker.tempWaypoint = null
      return zero()
    }
    return attractiveForce(robot.position, tracker.tempWaypoint, WAYPOINT_GAIN)
  }

  return zero()
}

// ─── Phase 11: Algorithm Comparison ──────────────────────────────────────────
//
// A* and RRT are deliberative planners — they pre-compute a geometric path from
// start to goal before the robot moves. The robot then follows that path using a
// simple waypoint-tracking controller (attractive force toward next waypoint +
// the same damping the APF robot uses). Running all three planners in the same
// environment lets users see the core tradeoff: APFs are reactive and need no
// pre-computation but get stuck at local minima; A* guarantees the shortest
// grid path but produces staircase artifacts; RRT handles cluttered workspaces
// probabilistically but gives a different path each run.

// Use params.attractiveGain (same as APF) so the comparison is fair on the attraction side.
// The only remaining difference is that followers have no repulsive forces — they follow
// the pre-planned path, while APF detours around obstacle influence zones in real time.
const FOLLOWER_WAYPOINT_ADVANCE = 0.8  // m — advance waypoint when this close
const FOLLOWER_TRAIL_MAX = 240
const RRT_TREE_RENDER_CAP = 600   // max tree edges sent to the canvas

export interface AlgorithmPlanInfo {
  planningTimeMs: number
  nodesExplored: number
  success: boolean
  plannedLength: number
  smoothness: number           // sum of interior angle changes in radians
  minClearance: number         // m from nearest obstacle surface
  path: Vector2[]
}

export interface AlgorithmFollowerInfo {
  robot: RobotState
  trail: Vector2[]
  traveledDist: number
  goalReached: boolean
  timeToGoal: number | null
}

export interface AlgorithmComparisonState {
  astar: { plan: AlgorithmPlanInfo | null; follower: AlgorithmFollowerInfo | null }
  rrt: {
    plan: AlgorithmPlanInfo | null
    follower: AlgorithmFollowerInfo | null
    tree: Array<{ from: Vector2; to: Vector2 }>
  }
}

// Mutable per-tick tracker — same pattern as RecoveryTracker, mutated in the
// rAF loop rather than copied so the hot path stays allocation-free.
interface PathFollowerTracker {
  robot: RobotState
  waypointIndex: number
  traveledDist: number
  goalReached: boolean
  timeToGoal: number | null
  startTime: number
  trail: Vector2[]
}

const seedFollower = (start: RobotState, simTime: number): PathFollowerTracker => ({
  robot: { position: { ...start.position }, velocity: { ...start.velocity } },
  waypointIndex: 0,
  traveledDist: 0,
  goalReached: false,
  timeToGoal: null,
  startTime: simTime,
  trail: [],
})

const stepFollower = (
  f: PathFollowerTracker,
  path: Vector2[],
  dt: number,
  params: SimulationParams,
  simTime: number,
): void => {
  if (f.goalReached || path.length === 0) return

  // Advance waypoint index past any waypoints we've already reached.
  while (f.waypointIndex < path.length - 1 &&
    distance(f.robot.position, path[f.waypointIndex]) < FOLLOWER_WAYPOINT_ADVANCE) {
    f.waypointIndex++
  }

  const target = path[f.waypointIndex]
  const isLast = f.waypointIndex === path.length - 1

  if (isLast && distance(f.robot.position, target) < GOAL_ARRIVAL_RADIUS) {
    f.goalReached = true
    f.timeToGoal = simTime - f.startTime
    return
  }

  const attrF = attractiveForce(f.robot.position, target, params.attractiveGain)
  const dampF = scale(f.robot.velocity, -params.damping)
  const total = add(attrF, dampF)
  const newVel = limit(add(f.robot.velocity, scale(total, dt)), params.maxSpeed)
  const posBefore = f.robot.position
  const newPos = add(posBefore, scale(newVel, dt))

  f.traveledDist += distance(posBefore, newPos)
  f.robot = { position: newPos, velocity: newVel }
  f.trail.push({ x: newPos.x, y: newPos.y })
  if (f.trail.length > FOLLOWER_TRAIL_MAX) f.trail.shift()
}

const MAX_RUN_HISTORY = 20
const initialLiveMetrics = (): LiveMetrics => ({
  elapsed: 0, pathLength: 0, straightLineDist: 0, efficiency: null,
  minObstacleDist: Infinity, timeInInfluenceZones: 0, trapCount: 0,
})

export interface SimulationLoop {
  state: SimulationState
  running: boolean
  timestep: number
  setTimestep: (dt: number) => void
  togglePlay: () => void
  step: () => void
  reset: () => void
  setGoal: (goal: Vector2 | null) => void
  toggleObstacle: (point: Vector2) => void
  params: SimulationParams
  setParams: (update: Partial<SimulationParams>) => void
  integrationMethod: IntegrationMethod
  setIntegrationMethod: (method: IntegrationMethod) => void
  comparisonTraces: ComparisonTrace[]
  localMinima: LocalMinimaDiagnostics
  recoveryMethod: RecoveryMethod
  setRecoveryMethod: (method: RecoveryMethod) => void
  recovery: RecoveryInfo
  // Phase 10 — research dashboard
  liveMetrics: LiveMetrics
  runHistory: RunMetrics[]
  clearHistory: () => void
  exportRunsJSON: () => void
  exportRunsCSV: () => void
  savedConfigs: ExperimentConfig[]
  saveConfig: (name: string) => void
  loadConfig: (config: ExperimentConfig) => void
  deleteConfig: (name: string) => void
  // Phase 11 — algorithm benchmarking
  algorithmComparison: AlgorithmComparisonState
  replanAlgorithms: () => void
  // Phase 12 — sensor simulation
  sensorConfig: SensorConfig
  setSensorConfig: (update: { lidar?: Partial<SensorConfig['lidar']> }) => void
  lidarHits: LidarHit[]
  // Phase 13 — occupancy grid mapping
  mappingEnabled: boolean
  setMappingEnabled: (v: boolean) => void
  occupancyGrid: OccupancyGrid
  occupancyGridCellSize: number
  setOccupancyGridCellSize: (v: number) => void
  clearOccupancyGrid: () => void
  occupancyGridStats: OccupancyGridStats
  // Phase 14 — uncertainty visualization
  uncertaintyConfig: UncertaintyConfig
  setUncertaintyConfig: (update: Partial<UncertaintyConfig>) => void
  uncertaintyStats: UncertaintyStats
  // Phase 15 — SLAM Lite
  slamEnabled: boolean
  setSlamEnabled: (v: boolean) => void
  slamConfig: SLAMConfig
  setSlamConfig: (update: Partial<SLAMConfig>) => void
  slamState: SLAMState
  clearSlam: () => void
}

// Fixed-timestep accumulator on top of rAF, so physics stay deterministic
// regardless of the display's refresh rate.
export const useSimulationLoop = (initialTimestep = 1 / 60): SimulationLoop => {
  const [state, setState] = useState<SimulationState>(createInitialState)
  const [running, setRunning] = useState(false)
  const [timestep, setTimestepState] = useState(initialTimestep)
  const [params, setParamsState] = useState<SimulationParams>(DEFAULT_PARAMS)
  const [integrationMethod, setIntegrationMethodState] = useState<IntegrationMethod>('semi-implicit-euler')
  const [localMinima, setLocalMinima] = useState<LocalMinimaDiagnostics>(initialDiagnostics)
  const [recoveryMethod, setRecoveryMethodState] = useState<RecoveryMethod>('none')
  const [recovery, setRecovery] = useState<RecoveryInfo>({
    active: false,
    tempWaypoint: null,
    stats: initialRecoveryStats(),
  })
  const [liveMetrics, setLiveMetrics] = useState<LiveMetrics>(initialLiveMetrics)
  const [runHistory, setRunHistory] = useState<RunMetrics[]>([])
  const [savedConfigs, setSavedConfigs] = useState<ExperimentConfig[]>(getSavedConfigs)
  // Phase 12 — sensor simulation
  const [sensorConfig, setSensorConfigState] = useState<SensorConfig>(DEFAULT_SENSOR_CONFIG)
  const [lidarHits, setLidarHitsState] = useState<LidarHit[]>([])
  // Phase 13 — occupancy grid mapping
  const [mappingEnabled, setMappingEnabledState] = useState(false)
  const [occupancyGridCellSize, setOccupancyGridCellSizeState] = useState(DEFAULT_GRID_CONFIG.cellSize)
  const occupancyGridRef = useRef<OccupancyGrid>(createGrid(DEFAULT_GRID_CONFIG))
  // Phase 14 — uncertainty visualization
  const [uncertaintyConfig, setUncertaintyConfigState] = useState<UncertaintyConfig>(DEFAULT_UNCERTAINTY_CONFIG)
  const covarianceRef = useRef<Covariance2>([...INITIAL_COVARIANCE])
  // Phase 15 — SLAM Lite
  const [slamEnabled, setSlamEnabledState] = useState(false)
  const [slamConfig, setSlamConfigState] = useState<SLAMConfig>(DEFAULT_SLAM_CONFIG)
  const slamStateRef = useRef<SLAMState>(createInitialSLAMState())
  const slamEnabledRef = useRef(slamEnabled)
  const slamConfigRef = useRef(slamConfig)

  // Phase 11 — plan info triggers re-renders on replan; follower state is derived from refs each render.
  const [astarPlanState, setAstarPlanState] = useState<AlgorithmPlanInfo | null>(null)
  const [rrtPlanState, setRrtPlanState] = useState<AlgorithmPlanInfo | null>(null)
  const [rrtTreeState, setRrtTreeState] = useState<Array<{ from: Vector2; to: Vector2 }>>([])
  const astarPlanRef = useRef<AlgorithmPlanInfo | null>(null)
  const rrtPlanRef = useRef<AlgorithmPlanInfo | null>(null)
  const astarFollowerRef = useRef<PathFollowerTracker | null>(null)
  const rrtFollowerRef = useRef<PathFollowerTracker | null>(null)

  const stateRef = useRef(state)
  const timestepRef = useRef(timestep)
  const runningRef = useRef(running)
  const paramsRef = useRef(params)
  const methodRef = useRef(integrationMethod)
  const recoveryMethodRef = useRef(recoveryMethod)
  const sensorConfigRef = useRef(sensorConfig)
  const lidarHitsRef = useRef<LidarHit[]>([])
  const mappingEnabledRef = useRef(mappingEnabled)
  const uncertaintyConfigRef = useRef(uncertaintyConfig)
  const accumulatorRef = useRef(0)
  const lastFrameRef = useRef<number | null>(null)

  // All three integrators run side by side from the same starting point every
  // tick — not just the selected one — so switching methods mid-run shows you
  // where that trajectory has *actually* drifted to, rather than restarting
  // the comparison from scratch. `state.robot` is just whichever of these
  // three is currently selected.
  const tracesRef = useRef(seedTraces(state.robot))
  const trailsRef = useRef(seedTrails())
  const trapTrackerRef = useRef(seedTrapTracker())
  const recoveryTrackerRef = useRef(seedRecoveryTracker())
  const metricsTrackerRef = useRef<MetricsTracker>(seedMetricsTracker(state.robot.position, 0, 0))
  const runIdRef = useRef(0)

  stateRef.current = state
  timestepRef.current = timestep
  runningRef.current = running
  paramsRef.current = params
  methodRef.current = integrationMethod
  recoveryMethodRef.current = recoveryMethod
  sensorConfigRef.current = sensorConfig
  mappingEnabledRef.current = mappingEnabled
  uncertaintyConfigRef.current = uncertaintyConfig
  slamEnabledRef.current = slamEnabled
  slamConfigRef.current = slamConfig
  astarPlanRef.current = astarPlanState
  rrtPlanRef.current = rrtPlanState

  // Advances every method's trace by one step using the same forces, and
  // records each one's new position in its trail (capped, FIFO).
  const advanceTraces = (goal: Vector2 | null, obstacles: Obstacle[], simParams: SimulationParams, dt: number, extraForce?: Vector2) => {
    const traces = tracesRef.current
    const trails = trailsRef.current
    const next = {} as Record<IntegrationMethod, RobotState>
    for (const method of ALL_METHODS) {
      const robot = integrateRobot(traces[method], goal, obstacles, simParams, dt, method, extraForce)
      next[method] = robot
      const trail = trails[method]
      trail.push(robot.position)
      if (trail.length > TRAIL_LENGTH) trail.shift()
    }
    tracesRef.current = next
  }

  // Builds a plan from planners and seeds fresh path-follower robots.
  // Called from setGoal / toggleObstacle / reset / replanAlgorithms.
  // All dependencies (planners, utils, setters) are stable so it's safe to
  // capture this at first render and reuse from inside useCallback closures.
  const doReplan = (start: Vector2, goal: Vector2, obstacles: Obstacle[], simTime: number) => {
    const astar = planAStar(start, goal, obstacles)
    const rrt = planRRT(start, goal, obstacles)

    const mkPlan = (r: { success: boolean; planningTimeMs: number; nodesExplored: number; path: Vector2[] }, obs: Obstacle[]): AlgorithmPlanInfo | null => {
      if (!r.success) return null
      return {
        planningTimeMs: r.planningTimeMs,
        nodesExplored: r.nodesExplored,
        success: true,
        plannedLength: computePathLength(r.path),
        smoothness: computeSmoothness(r.path),
        minClearance: computeMinClearance(r.path, obs),
        path: r.path,
      }
    }

    const astarPlan = mkPlan(astar, obstacles)
    const rrtPlan = mkPlan(rrt, obstacles)
    const seed: RobotState = { position: { ...start }, velocity: zero() }

    astarFollowerRef.current = astarPlan ? seedFollower(seed, simTime) : null
    rrtFollowerRef.current = rrtPlan ? seedFollower(seed, simTime) : null
    setAstarPlanState(astarPlan)
    setRrtPlanState(rrtPlan)
    setRrtTreeState((rrt as RRTResult).tree.slice(0, RRT_TREE_RENDER_CAP))
  }

  const clearPlans = () => {
    astarFollowerRef.current = null
    rrtFollowerRef.current = null
    setAstarPlanState(null)
    setRrtPlanState(null)
    setRrtTreeState([])
  }

  useEffect(() => {
    let frameId: number

    const tick = (now: number) => {
      frameId = requestAnimationFrame(tick)

      if (!runningRef.current) {
        lastFrameRef.current = now
        return
      }
      if (lastFrameRef.current === null) lastFrameRef.current = now

      accumulatorRef.current += (now - lastFrameRef.current) / 1000
      lastFrameRef.current = now

      const dt = timestepRef.current
      const { goal, obstacles } = stateRef.current
      let { time, stepCount } = stateRef.current

      // Phase 12 — update LiDAR scan once per frame (before substeps) so all
      // substeps share the same perceptual snapshot, matching a real sensor's
      // fixed update rate. effectiveObstacles is the perceived hit cloud when
      // sensor mode is on, or the true obstacle list otherwise.
      let effectiveObstacles: Obstacle[] = obstacles
      if (sensorConfigRef.current.lidar.enabled && obstacles.length > 0) {
        const hits = simulateLidar(stateRef.current.robot.position, 0, obstacles, sensorConfigRef.current.lidar)
        lidarHitsRef.current = hits
        setLidarHitsState(hits)
        effectiveObstacles = hitsToPerceivedObstacles(hits)
      } else if (!sensorConfigRef.current.lidar.enabled && lidarHitsRef.current.length > 0) {
        lidarHitsRef.current = []
        setLidarHitsState([])
      }

      let substeps = 0
      while (accumulatorRef.current >= dt && substeps < MAX_SUBSTEPS_PER_FRAME) {
        const rt = recoveryTrackerRef.current
        const posBefore = tracesRef.current[methodRef.current].position
        const extraForce =
          rt.wasTrapped && recoveryMethodRef.current !== 'none'
            ? recoveryForceFor(recoveryMethodRef.current, rt, tracesRef.current[methodRef.current], goal, effectiveObstacles, paramsRef.current, time)
            : undefined
        advanceTraces(goal, effectiveObstacles, paramsRef.current, dt, extraForce)
        const posAfter = tracesRef.current[methodRef.current].position
        // Step path-following robots (A* and RRT) alongside the APF robot.
        if (astarFollowerRef.current && astarPlanRef.current) {
          stepFollower(astarFollowerRef.current, astarPlanRef.current.path, dt, paramsRef.current, time + dt)
        }
        if (rrtFollowerRef.current && rrtPlanRef.current) {
          stepFollower(rrtFollowerRef.current, rrtPlanRef.current.path, dt, paramsRef.current, time + dt)
        }
        // Per-substep metrics: path length and obstacle proximity.
        const mt = metricsTrackerRef.current
        mt.pathLength += distance(posBefore, posAfter)
        for (const obs of obstacles) {
          const surf = Math.max(distance(posAfter, obs.position) - obs.radius, 0)
          if (surf < mt.minObstacleDist) mt.minObstacleDist = surf
          if (surf < paramsRef.current.obstacleInfluenceRadius) mt.timeInInfluenceZones += dt
        }
        time += dt
        stepCount += 1
        accumulatorRef.current -= dt
        substeps += 1
      }
      // Phase 13 — update occupancy grid from Phase 12 LiDAR hits once per frame.
      // Mapping intentionally requires sensor mode: the occupancy grid IS the
      // accumulated LiDAR scan history, not a separate magic-knowledge layer.
      if (
        mappingEnabledRef.current &&
        substeps > 0 &&
        sensorConfigRef.current.lidar.enabled &&
        lidarHitsRef.current.length > 0
      ) {
        updateGridFromHits(
          occupancyGridRef.current,
          tracesRef.current[methodRef.current].position,
          lidarHitsRef.current,
        )
      }

      if (substeps > 0) {
        const next: SimulationState = { robot: tracesRef.current[methodRef.current], goal, obstacles, time, stepCount }
        stateRef.current = next
        setState(next)
        const diag = evaluateLocalMinima(trapTrackerRef.current, time, next.robot, goal, effectiveObstacles, paramsRef.current)
        setLocalMinima(diag)
        const rt = recoveryTrackerRef.current
        const robotPos = next.robot.position
        // Genuine escape: robot has physically cleared the trap region.
        // Using distance (not gradient) prevents false positives where the
        // recovery force itself briefly lifts ∇U above epsilon on the next tick.
        if (rt.wasTrapped && rt.escapingFrom !== null && distance(robotPos, rt.escapingFrom) > ESCAPE_DISTANCE) {
          if (rt.trappedSince !== null && recoveryMethodRef.current !== 'none') {
            const m = recoveryMethodRef.current as Exclude<RecoveryMethod, 'none'>
            rt.stats[m].escapeCount += 1
            rt.stats[m].totalEscapeSeconds += time - rt.trappedSince
          }
          rt.wasTrapped = false
          rt.trappedSince = null
          rt.escapingFrom = null
          rt.tempWaypoint = null
        }
        // New trap confirmed — arm recovery only once per trap event.
        if (!rt.wasTrapped && diag.isTrapped && diag.trapPosition) {
          rt.wasTrapped = true
          rt.trappedSince = time
          rt.escapingFrom = diag.trapPosition
          if (recoveryMethodRef.current === 'waypoint' && goal) {
            rt.tempWaypoint = placeWaypoint(diag.trapPosition, goal, effectiveObstacles, paramsRef.current)
          }
        }
        setRecovery({ active: rt.wasTrapped && recoveryMethodRef.current !== 'none', tempWaypoint: rt.tempWaypoint, stats: { ...rt.stats } })
        // Trap metrics from diag (once per frame — close enough for stats).
        const mt2 = metricsTrackerRef.current
        if (diag.isTrapped && mt2.trappedSince === null) {
          mt2.trappedSince = time; mt2.trapCount += 1
        } else if (!diag.isTrapped && mt2.trappedSince !== null) {
          mt2.totalTrapTime += time - mt2.trappedSince; mt2.trappedSince = null
        }
        // Goal arrival: finalize run and add to history.
        if (!mt2.goalReached && goal && distance(robotPos, goal) < GOAL_ARRIVAL_RADIUS) {
          mt2.goalReached = true
          mt2.endTime = time
          if (mt2.trappedSince !== null) { mt2.totalTrapTime += time - mt2.trappedSince; mt2.trappedSince = null }
          const run = finalizeRun(mt2, goal, obstacles, methodRef.current, recoveryMethodRef.current, paramsRef.current)
          setRunHistory((prev) => [...prev.slice(-(MAX_RUN_HISTORY - 1)), run])
        }
        setLiveMetrics(liveSnapshot(mt2, goal, time))

        // Phase 14 — propagate position uncertainty based on motion, then correct
        // from LiDAR observations if the sensor is active and seeing features.
        // dt * substeps = total simulated time elapsed this rAF frame.
        const ucfg = uncertaintyConfigRef.current
        let cov = propagateCovariance(
          covarianceRef.current,
          next.robot.velocity,
          dt * substeps,
          ucfg,
        )
        if (
          ucfg.sensorCorrection &&
          sensorConfigRef.current.lidar.enabled &&
          lidarHitsRef.current.some((h) => h.hitObstacle && !h.dropped)
        ) {
          cov = correctCovariance(cov, dt * substeps, ucfg)
        }
        covarianceRef.current = cov

        // Phase 15 — SLAM Lite: update estimated position and landmark map.
        // Requires sensor mode for LiDAR hits (same constraint as Phase 13/14).
        if (
          slamEnabledRef.current &&
          sensorConfigRef.current.lidar.enabled &&
          lidarHitsRef.current.length > 0
        ) {
          slamStateRef.current = updateSLAM(
            slamStateRef.current,
            next.robot.velocity,
            lidarHitsRef.current,
            next.robot.position,  // true position — used only to compute posError
            dt * substeps,
            time,
            slamConfigRef.current,
          )
        }
      }
    }

    frameId = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(frameId)
  }, [])

  const togglePlay = useCallback(() => {
    setRunning((wasRunning) => {
      if (!wasRunning) {
        lastFrameRef.current = null
        accumulatorRef.current = 0
      }
      return !wasRunning
    })
  }, [])

  const step = useCallback(() => {
    setRunning(false)
    setState((prev) => {
      const dt = timestepRef.current
      const rt = recoveryTrackerRef.current
      const posBefore = tracesRef.current[methodRef.current].position
      const extraForce =
        rt.wasTrapped && recoveryMethodRef.current !== 'none'
          ? recoveryForceFor(recoveryMethodRef.current, rt, tracesRef.current[methodRef.current], prev.goal, prev.obstacles, paramsRef.current, prev.time)
          : undefined
      advanceTraces(prev.goal, prev.obstacles, paramsRef.current, dt, extraForce)
      const posAfter = tracesRef.current[methodRef.current].position
      const mt = metricsTrackerRef.current
      mt.pathLength += distance(posBefore, posAfter)
      for (const obs of prev.obstacles) {
        const surf = Math.max(distance(posAfter, obs.position) - obs.radius, 0)
        if (surf < mt.minObstacleDist) mt.minObstacleDist = surf
        if (surf < paramsRef.current.obstacleInfluenceRadius) mt.timeInInfluenceZones += dt
      }
      const next: SimulationState = {
        robot: tracesRef.current[methodRef.current],
        goal: prev.goal,
        obstacles: prev.obstacles,
        time: prev.time + dt,
        stepCount: prev.stepCount + 1,
      }
      stateRef.current = next
      const diag = evaluateLocalMinima(trapTrackerRef.current, next.time, next.robot, next.goal, next.obstacles, paramsRef.current)
      setLocalMinima(diag)
      if (rt.wasTrapped && rt.escapingFrom !== null && distance(next.robot.position, rt.escapingFrom) > ESCAPE_DISTANCE) {
        if (rt.trappedSince !== null && recoveryMethodRef.current !== 'none') {
          const m = recoveryMethodRef.current as Exclude<RecoveryMethod, 'none'>
          rt.stats[m].escapeCount += 1
          rt.stats[m].totalEscapeSeconds += next.time - rt.trappedSince
        }
        rt.wasTrapped = false
        rt.trappedSince = null
        rt.escapingFrom = null
        rt.tempWaypoint = null
      }
      if (!rt.wasTrapped && diag.isTrapped && diag.trapPosition) {
        rt.wasTrapped = true
        rt.trappedSince = next.time
        rt.escapingFrom = diag.trapPosition
        if (recoveryMethodRef.current === 'waypoint' && prev.goal) {
          rt.tempWaypoint = placeWaypoint(diag.trapPosition, prev.goal, prev.obstacles, paramsRef.current)
        }
      }
      setRecovery({ active: rt.wasTrapped && recoveryMethodRef.current !== 'none', tempWaypoint: rt.tempWaypoint, stats: { ...rt.stats } })
      if (diag.isTrapped && mt.trappedSince === null) {
        mt.trappedSince = next.time; mt.trapCount += 1
      } else if (!diag.isTrapped && mt.trappedSince !== null) {
        mt.totalTrapTime += next.time - mt.trappedSince; mt.trappedSince = null
      }
      if (!mt.goalReached && prev.goal && distance(posAfter, prev.goal) < GOAL_ARRIVAL_RADIUS) {
        mt.goalReached = true; mt.endTime = next.time
        if (mt.trappedSince !== null) { mt.totalTrapTime += next.time - mt.trappedSince; mt.trappedSince = null }
        const run = finalizeRun(mt, prev.goal, prev.obstacles, methodRef.current, recoveryMethodRef.current, paramsRef.current)
        setRunHistory((h) => [...h.slice(-(MAX_RUN_HISTORY - 1)), run])
      }
      setLiveMetrics(liveSnapshot(mt, prev.goal, next.time))
      return next
    })
  }, [])

  const setParams = useCallback((update: Partial<SimulationParams>) => {
    setParamsState((prev) => ({ ...prev, ...update }))
  }, [])

  const setRecoveryMethod = useCallback((method: RecoveryMethod) => {
    setRecoveryMethodState(method)
    recoveryMethodRef.current = method
    // Carry stats across method switches; reset ephemeral per-method state
    const prevStats = recoveryTrackerRef.current.stats
    recoveryTrackerRef.current = { ...seedRecoveryTracker(), stats: prevStats }
    setRecovery({ active: false, tempWaypoint: null, stats: { ...prevStats } })
  }, [])

  // Switching methods doesn't restart anything — it just changes which of the
  // three already-running traces gets to drive the visible robot. The jump
  // you see is real: that's how far this method's version of events has
  // diverged from the one you were just watching.
  const setIntegrationMethod = useCallback((method: IntegrationMethod) => {
    setIntegrationMethodState(method)
    setState((prev) => {
      const next = { ...prev, robot: tracesRef.current[method] }
      stateRef.current = next
      return next
    })
  }, [])

  const reset = useCallback(() => {
    setRunning(false)
    accumulatorRef.current = 0
    lastFrameRef.current = null
    setState((prev) => {
      const initial = { ...createInitialState(), goal: prev.goal, obstacles: prev.obstacles }
      // Finalize the outgoing run if anything happened during it.
      const mt = metricsTrackerRef.current
      if (mt.pathLength > 0.1) {
        const run = finalizeRun(mt, prev.goal, prev.obstacles, methodRef.current, recoveryMethodRef.current, paramsRef.current)
        setRunHistory((h) => [...h.slice(-(MAX_RUN_HISTORY - 1)), run])
      }
      runIdRef.current += 1
      tracesRef.current = seedTraces(initial.robot)
      trailsRef.current = seedTrails()
      trapTrackerRef.current = seedTrapTracker()
      recoveryTrackerRef.current = seedRecoveryTracker()
      metricsTrackerRef.current = seedMetricsTracker(initial.robot.position, 0, runIdRef.current)
      stateRef.current = initial
      setLocalMinima(initialDiagnostics())
      setRecovery({ active: false, tempWaypoint: null, stats: initialRecoveryStats() })
      setLiveMetrics(initialLiveMetrics())
      covarianceRef.current = [...INITIAL_COVARIANCE]
      slamStateRef.current = { ...createInitialSLAMState(), estPos: { ...initial.robot.position } }
      if (prev.goal) doReplan(initial.robot.position, prev.goal, prev.obstacles, 0)
      else clearPlans()
      return initial
    })
  }, [])

  // Reshaping the field invalidates any trap currently being timed — the
  // point where the gradient went flat may not even be a critical point of
  // the *new* field, so the clock has to restart rather than carry over.
  const setGoal = useCallback((goal: Vector2 | null) => {
    setState((prev) => {
      const next = { ...prev, goal }
      stateRef.current = next
      trapTrackerRef.current = seedTrapTracker()
      recoveryTrackerRef.current = seedRecoveryTracker()
      metricsTrackerRef.current = seedMetricsTracker(prev.robot.position, prev.time, runIdRef.current)
      setLocalMinima(initialDiagnostics())
      setRecovery({ active: false, tempWaypoint: null, stats: initialRecoveryStats() })
      setLiveMetrics(initialLiveMetrics())
      slamStateRef.current = { ...createInitialSLAMState(), estPos: { ...prev.robot.position } }
      if (goal) doReplan(prev.robot.position, goal, prev.obstacles, prev.time)
      else clearPlans()
      return next
    })
  }, [])

  const toggleObstacle = useCallback((point: Vector2) => {
    setState((prev) => {
      const hitIndex = prev.obstacles.findIndex((o) => distance(o.position, point) <= o.radius)
      const obstacles: Obstacle[] =
        hitIndex !== -1
          ? prev.obstacles.filter((_, i) => i !== hitIndex)
          : [...prev.obstacles, { position: point, radius: OBSTACLE_RADIUS }]
      const next = { ...prev, obstacles }
      stateRef.current = next
      trapTrackerRef.current = seedTrapTracker()
      recoveryTrackerRef.current = seedRecoveryTracker()
      metricsTrackerRef.current = seedMetricsTracker(prev.robot.position, prev.time, runIdRef.current)
      setLocalMinima(initialDiagnostics())
      setRecovery({ active: false, tempWaypoint: null, stats: initialRecoveryStats() })
      setLiveMetrics(initialLiveMetrics())
      slamStateRef.current = { ...createInitialSLAMState(), estPos: { ...prev.robot.position } }
      if (prev.goal) doReplan(prev.robot.position, prev.goal, obstacles, prev.time)
      return next
    })
  }, [])

  const clearHistory = useCallback(() => setRunHistory([]), [])
  const exportRunsJSON = () => exportJSON(runHistory)
  const exportRunsCSV = () => exportCSV(runHistory)

  const saveConfig = useCallback((name: string) => {
    persistConfig({
      name,
      goal: stateRef.current.goal,
      obstacles: stateRef.current.obstacles,
      params: { ...paramsRef.current },
      integrationMethod: methodRef.current,
      recoveryMethod: recoveryMethodRef.current,
    })
    setSavedConfigs(getSavedConfigs())
  }, [])

  const loadConfig = useCallback((config: ExperimentConfig) => {
    setRunning(false)
    accumulatorRef.current = 0
    lastFrameRef.current = null
    setParamsState(config.params)
    setIntegrationMethodState(config.integrationMethod as IntegrationMethod)
    setRecoveryMethodState(config.recoveryMethod as RecoveryMethod)
    recoveryMethodRef.current = config.recoveryMethod as RecoveryMethod
    runIdRef.current += 1
    setState(() => {
      const initial = { ...createInitialState(), goal: config.goal, obstacles: config.obstacles }
      tracesRef.current = seedTraces(initial.robot)
      trailsRef.current = seedTrails()
      trapTrackerRef.current = seedTrapTracker()
      recoveryTrackerRef.current = seedRecoveryTracker()
      metricsTrackerRef.current = seedMetricsTracker(initial.robot.position, 0, runIdRef.current)
      stateRef.current = initial
      setLocalMinima(initialDiagnostics())
      setRecovery({ active: false, tempWaypoint: null, stats: initialRecoveryStats() })
      setLiveMetrics(initialLiveMetrics())
      if (config.goal) doReplan(initial.robot.position, config.goal, config.obstacles, 0)
      else clearPlans()
      return initial
    })
  }, [])

  const deleteConfig = useCallback((name: string) => {
    removeConfig(name)
    setSavedConfigs(getSavedConfigs())
  }, [])

  // Ghosts for the two methods you're *not* currently driving with — the UI
  // draws these as faint trails so the divergence is something you can see,
  // not just take on faith.
  const comparisonTraces: ComparisonTrace[] = ALL_METHODS.filter((method) => method !== integrationMethod).map(
    (method) => ({ method, robot: tracesRef.current[method], trail: trailsRef.current[method] }),
  )

  // Derived each render from refs — follower state updates are visible without
  // separate React state because setState(next) fires every simulation frame.
  const algorithmComparison: AlgorithmComparisonState = {
    astar: {
      plan: astarPlanState,
      follower: astarFollowerRef.current
        ? {
            robot: astarFollowerRef.current.robot,
            trail: astarFollowerRef.current.trail,
            traveledDist: astarFollowerRef.current.traveledDist,
            goalReached: astarFollowerRef.current.goalReached,
            timeToGoal: astarFollowerRef.current.timeToGoal,
          }
        : null,
    },
    rrt: {
      plan: rrtPlanState,
      follower: rrtFollowerRef.current
        ? {
            robot: rrtFollowerRef.current.robot,
            trail: rrtFollowerRef.current.trail,
            traveledDist: rrtFollowerRef.current.traveledDist,
            goalReached: rrtFollowerRef.current.goalReached,
            timeToGoal: rrtFollowerRef.current.timeToGoal,
          }
        : null,
      tree: rrtTreeState,
    },
  }

  const replanAlgorithms = useCallback(() => {
    const { robot, goal, obstacles, time } = stateRef.current
    if (goal) doReplan(robot.position, goal, obstacles, time)
  }, [])

  const setSensorConfig = useCallback((update: { lidar?: Partial<SensorConfig['lidar']> }) => {
    setSensorConfigState((prev) => ({
      ...prev,
      lidar: { ...prev.lidar, ...(update.lidar ?? {}) },
    }))
  }, [])

  const setMappingEnabled = useCallback((v: boolean) => setMappingEnabledState(v), [])

  const setSlamEnabled = useCallback((v: boolean) => {
    setSlamEnabledState(v)
    if (v) {
      // Seed the ghost at the robot's current true position when SLAM is first
      // enabled so it doesn't start at origin mid-run and appear to teleport.
      slamStateRef.current = {
        ...createInitialSLAMState(),
        estPos: { ...stateRef.current.robot.position },
      }
    }
  }, [])

  const setSlamConfig = useCallback(
    (update: Partial<SLAMConfig>) =>
      setSlamConfigState((prev) => ({ ...prev, ...update })),
    [],
  )

  const clearSlam = useCallback(() => {
    slamStateRef.current = { ...createInitialSLAMState(), estPos: { ...stateRef.current.robot.position } }
  }, [])

  const setUncertaintyConfig = useCallback(
    (update: Partial<UncertaintyConfig>) =>
      setUncertaintyConfigState((prev) => ({ ...prev, ...update })),
    [],
  )

  const uncertaintyStats = covToStats(covarianceRef.current)

  const setOccupancyGridCellSize = useCallback((size: number) => {
    setOccupancyGridCellSizeState(size)
    // Rebuild the grid at the new resolution — clears accumulated data.
    occupancyGridRef.current = createGrid({ cellSize: size })
  }, [])

  const handleClearOccupancyGrid = useCallback(() => {
    clearGrid(occupancyGridRef.current)
  }, [])

  // Stats are cheap to compute each render (typed-array scan ~0.1 ms for ≤18 k cells).
  const occupancyGridStats = gridStats(occupancyGridRef.current)

  return {
    state,
    running,
    timestep,
    setTimestep: setTimestepState,
    togglePlay,
    step,
    reset,
    setGoal,
    toggleObstacle,
    params,
    setParams,
    integrationMethod,
    setIntegrationMethod,
    comparisonTraces,
    localMinima,
    recoveryMethod,
    setRecoveryMethod,
    recovery,
    liveMetrics,
    runHistory,
    clearHistory,
    exportRunsJSON,
    exportRunsCSV,
    savedConfigs,
    saveConfig,
    loadConfig,
    deleteConfig,
    algorithmComparison,
    replanAlgorithms,
    sensorConfig,
    setSensorConfig,
    lidarHits,
    mappingEnabled,
    setMappingEnabled,
    occupancyGrid: occupancyGridRef.current,
    occupancyGridCellSize,
    setOccupancyGridCellSize,
    clearOccupancyGrid: handleClearOccupancyGrid,
    occupancyGridStats,
    uncertaintyConfig,
    setUncertaintyConfig,
    uncertaintyStats,
    slamEnabled,
    setSlamEnabled,
    slamConfig,
    setSlamConfig,
    slamState: slamStateRef.current,
    clearSlam,
  }
}
