interface ControlPanelProps {
  running: boolean
  timestep: number
  time: number
  stepCount: number
  onTogglePlay: () => void
  onStep: () => void
  onReset: () => void
  onTimestepChange: (dt: number) => void
}

const MIN_HZ = 15
const MAX_HZ = 240

export const ControlPanel = ({
  running,
  timestep,
  time,
  stepCount,
  onTogglePlay,
  onStep,
  onReset,
  onTimestepChange,
}: ControlPanelProps) => {
  const hz = Math.round(1 / timestep)

  return (
    <div className="flex flex-col gap-3 p-3">
      <div className="flex gap-2">
        <button
          onClick={onTogglePlay}
          className="flex-1 bg-amber-400 px-3 py-1.5 text-sm font-medium text-slate-950 transition-colors hover:bg-amber-300"
        >
          {running ? 'Pause' : 'Start'}
        </button>
        <button
          onClick={onStep}
          className="border border-slate-700 px-3 py-1.5 text-sm text-slate-300 transition-colors hover:border-slate-500 hover:text-slate-100"
        >
          Step
        </button>
        <button
          onClick={onReset}
          className="border border-slate-700 px-3 py-1.5 text-sm text-slate-300 transition-colors hover:border-slate-500 hover:text-slate-100"
        >
          Reset
        </button>
      </div>

      <label className="flex flex-col gap-1.5 text-sm text-slate-400">
        <span className="flex justify-between">
          <span>Timestep</span>
          <span className="font-mono text-xs text-slate-500">
            {hz} Hz · {(timestep * 1000).toFixed(2)} ms
          </span>
        </span>
        <input
          type="range"
          min={MIN_HZ}
          max={MAX_HZ}
          step={1}
          value={hz}
          onChange={(e) => onTimestepChange(1 / Number(e.target.value))}
          className="accent-amber-400"
        />
      </label>

      <dl className="grid grid-cols-2 gap-x-4 gap-y-1 border-t border-slate-800 pt-3 font-mono text-xs text-slate-500">
        <dt>sim_time</dt>
        <dd className="text-right text-slate-300">{time.toFixed(2)}s</dd>
        <dt>steps</dt>
        <dd className="text-right text-slate-300">{stepCount}</dd>
      </dl>
    </div>
  )
}
