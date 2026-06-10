import type { LocalMinimaDiagnostics } from '@/hooks/useSimulationLoop'

interface LocalMinimaPanelProps {
  diagnostics: LocalMinimaDiagnostics
}

const TRAP_HOLD_SECONDS = 1.5

const statusFor = (d: LocalMinimaDiagnostics) => {
  if (d.isTrapped) {
    return {
      label: 'Trapped — local minimum',
      tone: 'border-rose-400 text-rose-300',
      detail: `∇U ≈ 0 here for ${d.stuckFor.toFixed(1)}s — critical point that isn't the goal.`,
    }
  }
  if (d.nearCriticalPoint && d.distanceToGoal !== null) {
    return {
      label: 'Near a critical point…',
      tone: 'border-amber-400 text-amber-300',
      detail: `Gradient flat for ${d.stuckFor.toFixed(1)}s — confirming trap vs. momentary pause (${TRAP_HOLD_SECONDS.toFixed(1)}s threshold).`,
    }
  }
  return {
    label: 'Descending the gradient',
    tone: 'border-slate-700 text-slate-400',
    detail: '‖∇U‖ is large — robot is being pulled toward a lower-potential region.',
  }
}

export const LocalMinimaPanel = ({ diagnostics }: LocalMinimaPanelProps) => {
  const status = statusFor(diagnostics)
  const progress = Math.min(diagnostics.stuckFor / TRAP_HOLD_SECONDS, 1)

  return (
    <div className="flex flex-col gap-3 p-3">
      <div className={`border px-3 py-1.5 text-sm ${status.tone}`}>{status.label}</div>

      <div className="flex flex-col gap-1">
        <span className="flex justify-between text-sm text-slate-400">
          <span>‖∇U‖ (net field force)</span>
          <span className="font-mono text-xs text-slate-500">{diagnostics.gradientMagnitude.toFixed(2)} N</span>
        </span>
        <span className="flex justify-between text-sm text-slate-400">
          <span>Distance to goal</span>
          <span className="font-mono text-xs text-slate-500">
            {diagnostics.distanceToGoal === null ? '—' : `${diagnostics.distanceToGoal.toFixed(2)} m`}
          </span>
        </span>
      </div>

      <div className="flex flex-col gap-1">
        <span className="flex justify-between text-[11px] uppercase tracking-[0.14em] text-slate-600">
          <span>Flat-gradient timer</span>
          <span className="font-mono">{diagnostics.stuckFor.toFixed(1)}s / {TRAP_HOLD_SECONDS.toFixed(1)}s</span>
        </span>
        <div className="h-1.5 w-full bg-slate-800">
          <div
            className={`h-full transition-[width] ${diagnostics.isTrapped ? 'bg-rose-400' : 'bg-amber-400'}`}
            style={{ width: `${progress * 100}%` }}
          />
        </div>
      </div>

      <p className="text-xs text-slate-600">{status.detail}</p>
      <p className="text-xs text-slate-600">
        Trap recipe: low attractive gain + high repulsive gain + goal behind a two-obstacle wall.
        The pulsing rose ring on the canvas marks the exact trap point.
      </p>
    </div>
  )
}
