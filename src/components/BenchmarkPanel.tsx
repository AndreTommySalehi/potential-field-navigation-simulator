import type { AlgorithmComparisonState } from '@/hooks/useSimulationLoop'
import type { LiveMetrics, RunMetrics } from '@/sim/metrics'

interface BenchmarkPanelProps {
  comparison: AlgorithmComparisonState
  apfLive: LiveMetrics
  apfLastRun: RunMetrics | null
}

const fmt1 = (n: number) => n.toFixed(1)
const fmt2 = (n: number) => n.toFixed(2)
const fmtMs = (n: number) => `${n.toFixed(0)} ms`
const fmtOr = (n: number | null | undefined, fn: (v: number) => string, fallback = '—') =>
  n == null || n === Infinity ? fallback : fn(n)

const ALG_COLORS = {
  apf: 'text-sky-400',
  astar: 'text-cyan-400',
  rrt: 'text-amber-400',
} as const

const ALG_LABELS = { apf: 'APF', astar: 'A*', rrt: 'RRT' }

interface Col {
  key: 'apf' | 'astar' | 'rrt'
  planTime: string
  plannedLen: string
  traveled: string
  timeToGoal: string
  smoothness: string
  clearance: string
  status: string
}

export const BenchmarkPanel = ({ comparison, apfLive, apfLastRun }: BenchmarkPanelProps) => {
  const { astar, rrt } = comparison

  const apfStatus = apfLastRun?.goalReached
    ? `reached (${apfLastRun.timeToGoal?.toFixed(1) ?? '?'}s)`
    : apfLive.pathLength > 0.1
      ? 'running'
      : 'waiting'

  const cols: Col[] = [
    {
      key: 'apf',
      planTime: '—',
      plannedLen: '—',
      traveled: `${fmt1(apfLive.pathLength)} m`,
      timeToGoal: apfLastRun?.goalReached
        ? fmtOr(apfLastRun.timeToGoal, (v) => `${fmt1(v)} s`)
        : apfLive.elapsed > 0
          ? `${fmt1(apfLive.elapsed)} s…`
          : '—',
      smoothness: '—',
      clearance: fmtOr(apfLive.minObstacleDist === Infinity ? null : apfLive.minObstacleDist, (v) => `${fmt2(v)} m`),
      status: apfStatus,
    },
    {
      key: 'astar',
      planTime: astar.plan ? fmtMs(astar.plan.planningTimeMs) : '—',
      plannedLen: astar.plan ? `${fmt1(astar.plan.plannedLength)} m` : '—',
      traveled: astar.follower ? `${fmt1(astar.follower.traveledDist)} m` : '—',
      timeToGoal: astar.follower?.goalReached
        ? fmtOr(astar.follower.timeToGoal, (v) => `${fmt1(v)} s`)
        : astar.follower
          ? 'running…'
          : '—',
      smoothness: astar.plan ? `${fmt2(astar.plan.smoothness)} rad` : '—',
      clearance: astar.plan ? `${fmt2(astar.plan.minClearance)} m` : '—',
      status: astar.plan
        ? astar.follower?.goalReached
          ? 'reached'
          : astar.follower
            ? 'following'
            : 'planned'
        : '—',
    },
    {
      key: 'rrt',
      planTime: rrt.plan ? fmtMs(rrt.plan.planningTimeMs) : '—',
      plannedLen: rrt.plan ? `${fmt1(rrt.plan.plannedLength)} m` : '—',
      traveled: rrt.follower ? `${fmt1(rrt.follower.traveledDist)} m` : '—',
      timeToGoal: rrt.follower?.goalReached
        ? fmtOr(rrt.follower.timeToGoal, (v) => `${fmt1(v)} s`)
        : rrt.follower
          ? 'running…'
          : '—',
      smoothness: rrt.plan ? `${fmt2(rrt.plan.smoothness)} rad` : '—',
      clearance: rrt.plan ? `${fmt2(rrt.plan.minClearance)} m` : '—',
      status: rrt.plan
        ? rrt.follower?.goalReached
          ? 'reached'
          : rrt.follower
            ? 'following'
            : 'planned'
        : '—',
    },
  ]

  const rows: { label: string; key: keyof Omit<Col, 'key'> }[] = [
    { label: 'Plan time', key: 'planTime' },
    { label: 'Planned len', key: 'plannedLen' },
    { label: 'Traveled', key: 'traveled' },
    { label: 'Time to goal', key: 'timeToGoal' },
    { label: 'Smoothness ↓', key: 'smoothness' },
    { label: 'Clearance ↑', key: 'clearance' },
    { label: 'Status', key: 'status' },
  ]

  return (
    <div className="p-3">
      <div className="mb-2 grid grid-cols-4 gap-1 text-[11px] uppercase tracking-[0.12em]">
        <span className="text-slate-600">Metric</span>
        {cols.map((c) => (
          <span key={c.key} className={`text-center font-medium ${ALG_COLORS[c.key]}`}>
            {ALG_LABELS[c.key]}
          </span>
        ))}
      </div>

      <div className="flex flex-col gap-0.5">
        {rows.map(({ label, key }) => (
          <div key={key} className="grid grid-cols-4 gap-1 border-t border-slate-800/60 py-0.5 text-xs">
            <span className="text-slate-600">{label}</span>
            {cols.map((c) => (
              <span key={c.key} className="text-center font-mono text-slate-400">
                {c[key]}
              </span>
            ))}
          </div>
        ))}
      </div>

      {(astar.plan || rrt.plan) && (
        <p className="mt-3 text-[10px] text-slate-700">
          {astar.plan ? `A* explored ${astar.plan.nodesExplored} nodes. ` : ''}
          {rrt.plan ? `RRT grew ${rrt.plan.nodesExplored} nodes.` : ''}
        </p>
      )}

      {!astar.plan && !rrt.plan && (
        <p className="mt-3 text-xs text-slate-600">Place a goal to trigger planning.</p>
      )}
    </div>
  )
}
