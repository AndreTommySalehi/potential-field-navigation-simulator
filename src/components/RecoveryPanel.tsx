import type { RecoveryInfo, RecoveryMethod } from '@/hooks/useSimulationLoop'

const METHOD_INFO: Record<RecoveryMethod, { label: string; blurb: string }> = {
  none: {
    label: 'None',
    blurb: 'No recovery — robot stays trapped at the local minimum.',
  },
  'random-walk': {
    label: 'Random Walk',
    blurb: 'Applies a random impulse every 0.4s. Stochastic — escapes any trap eventually, but path is chaotic.',
  },
  'virtual-force': {
    label: 'Virtual Force',
    blurb: 'Pushes perpendicular to the trap→goal axis, exploiting saddle-point geometry. Deterministic.',
  },
  waypoint: {
    label: 'Waypoint',
    blurb: 'Plants a temporary sub-goal offset from the trap to pull the robot clear. Most path-efficient.',
  },
}

const STAT_METHODS: Exclude<RecoveryMethod, 'none'>[] = ['random-walk', 'virtual-force', 'waypoint']

interface RecoveryPanelProps {
  method: RecoveryMethod
  onMethodChange: (m: RecoveryMethod) => void
  recovery: RecoveryInfo
}

export const RecoveryPanel = ({ method, onMethodChange, recovery }: RecoveryPanelProps) => {
  const info = METHOD_INFO[method]

  return (
    <div className="flex flex-col gap-3 p-3">
      <div className="grid grid-cols-2 gap-1">
        {(Object.keys(METHOD_INFO) as RecoveryMethod[]).map((m) => (
          <button
            key={m}
            onClick={() => onMethodChange(m)}
            className={`px-2 py-1.5 text-left text-xs transition-colors ${
              method === m
                ? 'bg-emerald-900 text-emerald-300 outline outline-1 outline-emerald-600'
                : 'bg-slate-800 text-slate-400 hover:bg-slate-700 hover:text-slate-200'
            }`}
          >
            {METHOD_INFO[m].label}
          </button>
        ))}
      </div>

      <p className="text-xs text-slate-500">{info.blurb}</p>

      {method !== 'none' && (
        <div
          className={`px-3 py-1.5 text-sm ${
            recovery.active
              ? 'border border-emerald-500 text-emerald-300'
              : 'border border-slate-700 text-slate-500'
          }`}
        >
          {recovery.active ? 'Recovery active — escaping trap' : 'Standby — no trap detected'}
        </div>
      )}

      <div className="flex flex-col gap-1">
        <span className="text-[11px] uppercase tracking-[0.14em] text-slate-600">Escape stats</span>
        <table className="w-full text-xs">
          <thead>
            <tr className="text-left text-slate-600">
              <th className="pb-1 font-normal">Method</th>
              <th className="pb-1 text-right font-normal">Escapes</th>
              <th className="pb-1 text-right font-normal">Avg time</th>
            </tr>
          </thead>
          <tbody>
            {STAT_METHODS.map((m) => {
              const s = recovery.stats[m]
              const avg = s.escapeCount > 0 ? s.totalEscapeSeconds / s.escapeCount : null
              return (
                <tr key={m} className={`border-t border-slate-800 ${method === m ? 'text-emerald-400' : 'text-slate-500'}`}>
                  <td className="py-1">{METHOD_INFO[m].label}</td>
                  <td className="py-1 text-right font-mono">{s.escapeCount}</td>
                  <td className="py-1 text-right font-mono">{avg !== null ? `${avg.toFixed(1)}s` : '—'}</td>
                </tr>
              )
            })}
          </tbody>
        </table>
      </div>
    </div>
  )
}
