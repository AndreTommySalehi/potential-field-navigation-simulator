import { distance } from '@/lib/vector'
import { INTEGRATION_METHODS, type IntegrationMethod, type RobotState } from '@/sim/simulationEngine'
import type { ComparisonTrace } from '@/hooks/useSimulationLoop'

interface IntegrationPanelProps {
  method: IntegrationMethod
  onMethodChange: (method: IntegrationMethod) => void
  activeRobot: RobotState
  comparisonTraces: ComparisonTrace[]
}

const METHOD_TEXT: Record<IntegrationMethod, string> = {
  euler: 'text-orange-300',
  'semi-implicit-euler': 'text-fuchsia-300',
  rk4: 'text-emerald-300',
}

const METHOD_BORDER: Record<IntegrationMethod, string> = {
  euler: 'border-orange-400',
  'semi-implicit-euler': 'border-fuchsia-400',
  rk4: 'border-emerald-400',
}

export const IntegrationPanel = ({ method, onMethodChange, activeRobot, comparisonTraces }: IntegrationPanelProps) => {
  const active = INTEGRATION_METHODS.find((m) => m.id === method)!

  return (
    <div className="flex flex-col gap-3 p-3">
      <div className="flex flex-col gap-1.5">
        {INTEGRATION_METHODS.map((m) => (
          <button
            key={m.id}
            onClick={() => onMethodChange(m.id)}
            className={`border px-3 py-1.5 text-left text-sm transition-colors ${
              method === m.id
                ? `${METHOD_BORDER[m.id]} ${METHOD_TEXT[m.id]}`
                : 'border-slate-700 text-slate-400 hover:border-slate-500 hover:text-slate-200'
            }`}
          >
            {m.label}
          </button>
        ))}
      </div>
      <p className="text-xs text-slate-600">{active.blurb}</p>

      <div className="flex flex-col gap-1 border-t border-slate-800 pt-3">
        <span className="text-[11px] uppercase tracking-[0.14em] text-slate-600">Drift from here</span>
        {comparisonTraces.map((trace) => {
          const spec = INTEGRATION_METHODS.find((m) => m.id === trace.method)!
          const drift = distance(activeRobot.position, trace.robot.position)
          return (
            <span key={trace.method} className="flex justify-between text-sm">
              <span className={METHOD_TEXT[trace.method]}>{spec.label}</span>
              <span className="font-mono text-xs text-slate-500">{drift.toFixed(2)} m</span>
            </span>
          )
        })}
        <span className="text-xs text-slate-600">
          How far each integrator has diverged since last reset; faint canvas trails trace the actual paths.
        </span>
      </div>
    </div>
  )
}
