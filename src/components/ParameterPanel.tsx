import type { SimulationParams } from '@/sim/forces'

interface ParameterPanelProps {
  params: SimulationParams
  onChange: (update: Partial<SimulationParams>) => void
}

interface SliderSpec {
  key: keyof SimulationParams
  label: string
  min: number
  max: number
  step: number
  hint: string
  format?: (value: number) => string
}

const SLIDERS: SliderSpec[] = [
  {
    key: 'attractiveGain',
    label: 'attractive gain (k)',
    min: 0,
    max: 5,
    step: 0.1,
    hint: 'Spring stiffness toward the goal.',
  },
  {
    key: 'repulsiveGain',
    label: 'repulsive gain (η)',
    min: 0,
    max: 10,
    step: 0.1,
    hint: 'Push strength from obstacle surfaces.',
  },
  {
    key: 'obstacleInfluenceRadius',
    label: 'influence radius (Q*)',
    min: 0.2,
    max: 4,
    step: 0.1,
    hint: 'How far the repulsive field extends past an obstacle.',
  },
  {
    key: 'damping',
    label: 'damping (b)',
    min: 0,
    max: 5,
    step: 0.1,
    hint: 'Opposes velocity — prevents oscillation at goal.',
  },
  {
    key: 'maxSpeed',
    label: 'max speed',
    min: 0.5,
    max: 12,
    step: 0.5,
    hint: 'Hard speed cap.',
    format: (v) => `${v.toFixed(1)} m/s`,
  },
]

export const ParameterPanel = ({ params, onChange }: ParameterPanelProps) => (
  <div className="flex flex-col gap-3 p-3">
    {SLIDERS.map(({ key, label, min, max, step, hint, format }) => {
      const value = params[key]
      return (
        <label key={key} className="flex flex-col gap-1 text-sm text-slate-400">
          <span className="flex justify-between">
            <span className="font-mono text-xs text-slate-300">{label}</span>
            <span className="font-mono text-xs text-slate-500">
              {format ? format(value) : value.toFixed(2)}
            </span>
          </span>
          <input
            type="range"
            min={min}
            max={max}
            step={step}
            value={value}
            onChange={(e) => onChange({ [key]: Number(e.target.value) })}
            className="accent-amber-400"
          />
          <span className="text-xs text-slate-600">{hint}</span>
        </label>
      )
    })}
  </div>
)
