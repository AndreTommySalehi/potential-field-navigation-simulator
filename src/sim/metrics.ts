import type { Vector2 } from '@/lib/vector'
import { distance } from '@/lib/vector'
import type { Obstacle, SimulationParams } from './forces'

// Must match GOAL_ARRIVAL_RADIUS in useSimulationLoop.ts.
const GOAL_ARRIVAL_RADIUS = 1

export interface LiveMetrics {
  elapsed: number            // sim-seconds since last reset
  pathLength: number         // total distance traveled (m)
  straightLineDist: number   // straight-line start→goal distance (m)
  efficiency: number | null  // straight / path — 1.0 = perfect bee-line; null before meaningful path
  minObstacleDist: number    // closest approach to any obstacle surface (m); Infinity if none
  timeInInfluenceZones: number // sim-seconds inside any obstacle's Q* zone
  trapCount: number          // distinct trap events so far this run
}

export interface RunMetrics {
  id: number
  goalReached: boolean
  timeToGoal: number | null   // sim-seconds; null if goal never reached
  pathLength: number
  straightLineDist: number
  efficiency: number | null
  minObstacleDist: number
  timeInInfluenceZones: number
  trapCount: number
  totalTrapTime: number
  obstacleCount: number
  // string rather than the union types — keeps this file free of hook imports
  integrationMethod: string
  recoveryMethod: string
  params: SimulationParams
}

export interface ExperimentConfig {
  name: string
  goal: Vector2 | null
  obstacles: Obstacle[]
  params: SimulationParams
  integrationMethod: string
  recoveryMethod: string
}

// ── Mutable per-run tracker (kept in a ref, mutated each physics step) ────────

export interface MetricsTracker {
  runId: number
  startPosition: Vector2
  startTime: number
  pathLength: number
  minObstacleDist: number
  timeInInfluenceZones: number
  trapCount: number
  totalTrapTime: number
  trappedSince: number | null // sim-time when current trap began
  goalReached: boolean
  endTime: number | null
}

export const seedMetricsTracker = (startPosition: Vector2, startTime: number, runId: number): MetricsTracker => ({
  runId,
  startPosition,
  startTime,
  pathLength: 0,
  minObstacleDist: Infinity,
  timeInInfluenceZones: 0,
  trapCount: 0,
  totalTrapTime: 0,
  trappedSince: null,
  goalReached: false,
  endTime: null,
})

// Call once per physics substep to keep obstacle-proximity accurate.
// Returns true the first time goal arrival is detected (transition only).
export const stepMetrics = (
  tracker: MetricsTracker,
  stepDist: number,            // distance traveled in this substep
  position: Vector2,
  goal: Vector2 | null,
  obstacles: Obstacle[],
  params: SimulationParams,
  isTrapped: boolean,
  simTime: number,
  dt: number,
): boolean => {
  tracker.pathLength += stepDist

  for (const obs of obstacles) {
    const surf = Math.max(distance(position, obs.position) - obs.radius, 0)
    if (surf < tracker.minObstacleDist) tracker.minObstacleDist = surf
    if (surf < params.obstacleInfluenceRadius) tracker.timeInInfluenceZones += dt
  }

  if (isTrapped && tracker.trappedSince === null) {
    tracker.trappedSince = simTime
    tracker.trapCount += 1
  } else if (!isTrapped && tracker.trappedSince !== null) {
    tracker.totalTrapTime += simTime - tracker.trappedSince
    tracker.trappedSince = null
  }

  if (!tracker.goalReached && goal && distance(position, goal) < GOAL_ARRIVAL_RADIUS) {
    tracker.goalReached = true
    tracker.endTime = simTime
    if (tracker.trappedSince !== null) {
      tracker.totalTrapTime += simTime - tracker.trappedSince
      tracker.trappedSince = null
    }
    return true // goal just reached
  }
  return false
}

export const liveSnapshot = (tracker: MetricsTracker, goal: Vector2 | null, simTime: number): LiveMetrics => {
  const straightLineDist = goal ? distance(tracker.startPosition, goal) : 0
  return {
    elapsed: simTime - tracker.startTime,
    pathLength: tracker.pathLength,
    straightLineDist,
    efficiency: tracker.pathLength > 0.1 ? straightLineDist / tracker.pathLength : null,
    minObstacleDist: tracker.minObstacleDist,
    timeInInfluenceZones: tracker.timeInInfluenceZones,
    trapCount: tracker.trapCount,
  }
}

export const finalizeRun = (
  tracker: MetricsTracker,
  goal: Vector2 | null,
  obstacles: Obstacle[],
  integrationMethod: string,
  recoveryMethod: string,
  params: SimulationParams,
): RunMetrics => {
  const straightLineDist = goal ? distance(tracker.startPosition, goal) : 0
  const timeToGoal = tracker.goalReached && tracker.endTime !== null ? tracker.endTime - tracker.startTime : null
  const efficiency =
    tracker.goalReached && tracker.pathLength > 0.1 ? straightLineDist / tracker.pathLength : null
  let totalTrapTime = tracker.totalTrapTime
  if (tracker.trappedSince !== null) {
    const closeAt = tracker.endTime ?? (tracker.startTime + (tracker.pathLength > 0 ? 1 : 0))
    totalTrapTime += closeAt - tracker.trappedSince
  }
  return {
    id: tracker.runId,
    goalReached: tracker.goalReached,
    timeToGoal,
    pathLength: tracker.pathLength,
    straightLineDist,
    efficiency,
    minObstacleDist: tracker.minObstacleDist,
    timeInInfluenceZones: tracker.timeInInfluenceZones,
    trapCount: tracker.trapCount,
    totalTrapTime,
    obstacleCount: obstacles.length,
    integrationMethod,
    recoveryMethod,
    params: { ...params },
  }
}

// ── Experiment config persistence (localStorage) ──────────────────────────────

const CONFIGS_KEY = 'pfnav-experiment-configs'

export const getSavedConfigs = (): ExperimentConfig[] => {
  try {
    return JSON.parse(localStorage.getItem(CONFIGS_KEY) ?? '[]') as ExperimentConfig[]
  } catch {
    return []
  }
}

export const persistConfig = (config: ExperimentConfig): void => {
  const rest = getSavedConfigs().filter((c) => c.name !== config.name)
  localStorage.setItem(CONFIGS_KEY, JSON.stringify([...rest, config]))
}

export const removeConfig = (name: string): void => {
  localStorage.setItem(CONFIGS_KEY, JSON.stringify(getSavedConfigs().filter((c) => c.name !== name)))
}

// ── Data export ───────────────────────────────────────────────────────────────

const triggerDownload = (content: string, filename: string, mime: string) => {
  const blob = new Blob([content], { type: mime })
  const url = URL.createObjectURL(blob)
  const a = document.createElement('a')
  a.href = url
  a.download = filename
  a.click()
  URL.revokeObjectURL(url)
}

const CSV_HEADERS = [
  'id', 'goalReached', 'timeToGoal', 'pathLength', 'straightLineDist', 'efficiency',
  'minObstacleDist', 'timeInInfluenceZones', 'trapCount', 'totalTrapTime', 'obstacleCount',
  'integrationMethod', 'recoveryMethod',
  'attractiveGain', 'repulsiveGain', 'obstacleInfluenceRadius', 'damping', 'maxSpeed',
]

export const exportJSON = (runs: RunMetrics[]) => {
  triggerDownload(JSON.stringify(runs, null, 2), 'pfnav-runs.json', 'application/json')
}

export const exportCSV = (runs: RunMetrics[]) => {
  const rows = runs.map((r) => [
    r.id, r.goalReached, r.timeToGoal ?? '', r.pathLength.toFixed(3), r.straightLineDist.toFixed(3),
    r.efficiency !== null ? r.efficiency.toFixed(3) : '',
    r.minObstacleDist === Infinity ? '' : r.minObstacleDist.toFixed(3),
    r.timeInInfluenceZones.toFixed(3), r.trapCount, r.totalTrapTime.toFixed(3), r.obstacleCount,
    r.integrationMethod, r.recoveryMethod,
    r.params.attractiveGain, r.params.repulsiveGain, r.params.obstacleInfluenceRadius,
    r.params.damping, r.params.maxSpeed,
  ].join(','))
  triggerDownload([CSV_HEADERS.join(','), ...rows].join('\n'), 'pfnav-runs.csv', 'text/csv')
}
