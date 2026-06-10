import type { LiveMetrics, RunMetrics } from '@/sim/metrics'

interface MetricsPanelProps {
  live: LiveMetrics
  history: RunMetrics[]
  onClearHistory: () => void
  onExportJSON: () => void
  onExportCSV: () => void
}

const fmt = (n: number, decimals = 2) => n.toFixed(decimals)
const fmtOpt = (n: number | null, decimals = 2, suffix = '') =>
  n === null ? '—' : `${n.toFixed(decimals)}${suffix}`

const Row = ({ label, value }: { label: string; value: string }) => (
  <span className="flex justify-between text-sm text-slate-400">
    <span>{label}</span>
    <span className="font-mono text-xs text-slate-500">{value}</span>
  </span>
)

export const MetricsPanel = ({ live, history, onClearHistory, onExportJSON, onExportCSV }: MetricsPanelProps) => {
  const effPct = live.efficiency !== null ? `${(live.efficiency * 100).toFixed(0)}%` : '—'
  const nearestObs = live.minObstacleDist === Infinity ? '—' : `${fmt(live.minObstacleDist)} m`

  return (
    <>
      <div className="flex flex-col gap-2 border-b border-slate-800 p-3">
        <span className="text-[11px] uppercase tracking-[0.14em] text-slate-600">Current run</span>
        <Row label="Elapsed" value={`${fmt(live.elapsed)} s`} />
        <Row label="Path length" value={`${fmt(live.pathLength)} m`} />
        <Row label="Direct dist" value={`${fmt(live.straightLineDist)} m`} />
        <Row label="Efficiency" value={effPct} />
        <Row label="Nearest obstacle" value={nearestObs} />
        <Row label="Time in Q* zones" value={`${fmt(live.timeInInfluenceZones)} s`} />
        <Row label="Trap events" value={String(live.trapCount)} />
      </div>

      <div className="flex gap-2 border-b border-slate-800 p-3">
        <button
          onClick={onExportJSON}
          disabled={history.length === 0}
          className="flex-1 bg-slate-800 px-2 py-1.5 text-xs text-slate-300 hover:bg-slate-700 disabled:cursor-not-allowed disabled:text-slate-600"
        >
          JSON
        </button>
        <button
          onClick={onExportCSV}
          disabled={history.length === 0}
          className="flex-1 bg-slate-800 px-2 py-1.5 text-xs text-slate-300 hover:bg-slate-700 disabled:cursor-not-allowed disabled:text-slate-600"
        >
          CSV
        </button>
        <button
          onClick={onClearHistory}
          disabled={history.length === 0}
          className="flex-1 bg-slate-800 px-2 py-1.5 text-xs text-slate-400 hover:bg-slate-700 hover:text-rose-400 disabled:cursor-not-allowed disabled:text-slate-600"
        >
          Clear
        </button>
      </div>

      <div className="p-3">
        <span className="text-[11px] uppercase tracking-[0.14em] text-slate-600">
          Run history ({history.length})
        </span>
        {history.length === 0 ? (
          <p className="mt-2 text-xs text-slate-600">Reach a goal to record the first run.</p>
        ) : (
          <div className="mt-2 max-h-52 overflow-y-auto">
            <table className="w-full text-xs">
              <thead>
                <tr className="text-left text-slate-600">
                  <th className="pb-1 font-normal">#</th>
                  <th className="pb-1 font-normal">Time</th>
                  <th className="pb-1 font-normal">Length</th>
                  <th className="pb-1 font-normal">Eff</th>
                  <th className="pb-1 font-normal">Traps</th>
                </tr>
              </thead>
              <tbody>
                {[...history].reverse().map((r) => (
                  <tr key={r.id} className="border-t border-slate-800 text-slate-400">
                    <td className="py-0.5 font-mono text-slate-600">{r.id}</td>
                    <td className="py-0.5 font-mono">
                      {r.goalReached ? (
                        <span className="text-emerald-500">{fmtOpt(r.timeToGoal, 1, 's')}</span>
                      ) : (
                        <span className="text-slate-600">✗</span>
                      )}
                    </td>
                    <td className="py-0.5 font-mono">{fmt(r.pathLength, 1)}m</td>
                    <td className="py-0.5 font-mono">{fmtOpt(r.efficiency !== null ? r.efficiency * 100 : null, 0, '%')}</td>
                    <td className="py-0.5 font-mono">{r.trapCount}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        )}
      </div>
    </>
  )
}
