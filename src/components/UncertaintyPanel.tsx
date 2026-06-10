import type { UncertaintyConfig, UncertaintyStats } from '@/sim/uncertainty'

interface UncertaintyPanelProps {
  config: UncertaintyConfig
  onChange: (update: Partial<UncertaintyConfig>) => void
  stats: UncertaintyStats
  showEllipse: boolean
  onShowEllipseChange: (v: boolean) => void
  sigmaLevel: number
  onSigmaLevelChange: (v: number) => void
}

const SIGMA_LEVELS = [1, 2, 3] as const

export const UncertaintyPanel = ({
  config,
  onChange,
  stats,
  showEllipse,
  onShowEllipseChange,
  sigmaLevel,
  onSigmaLevelChange,
}: UncertaintyPanelProps) => {
  const angleDeg = ((stats.angle * 180) / Math.PI).toFixed(1)

  return (
    <div className="flex flex-col gap-3 p-3">
      {/* Ellipse display controls */}
      <div className="flex items-center justify-between gap-2">
        <div className="flex items-center gap-2">
          <span className="text-xs text-slate-400">Ellipse</span>
          <button
            onClick={() => onShowEllipseChange(!showEllipse)}
            className={`border px-2 py-0.5 text-xs transition-colors ${
              showEllipse
                ? 'border-indigo-500 text-indigo-400'
                : 'border-slate-700 text-slate-500 hover:border-slate-600 hover:text-slate-400'
            }`}
          >
            {showEllipse ? 'on' : 'off'}
          </button>
        </div>
        <div className="flex items-center gap-1">
          {SIGMA_LEVELS.map((n) => (
            <button
              key={n}
              onClick={() => onSigmaLevelChange(n)}
              className={`border px-1.5 py-0.5 text-xs transition-colors ${
                sigmaLevel === n
                  ? 'border-indigo-500 text-indigo-400'
                  : 'border-slate-700 text-slate-600 hover:text-slate-400'
              }`}
            >
              {n}σ
            </button>
          ))}
        </div>
      </div>

      <p className="text-xs text-slate-600">
        Confidence ellipse around the robot's estimated position. The ellipse grows
        with dead-reckoning drift and shrinks when LiDAR observes known features.
      </p>

      {/* Noise model */}
      <div className="flex flex-col gap-2 border-t border-slate-800 pt-2">
        <span className="text-[11px] uppercase tracking-[0.14em] text-slate-600">Noise model</span>

        <label className="flex flex-col gap-1 text-xs text-slate-400">
          <span className="flex justify-between">
            <span>Process noise</span>
            <span className="font-mono text-slate-500">{config.processNoise.toFixed(3)} m²/s</span>
          </span>
          <input
            type="range" min={0} max={0.05} step={0.001}
            value={config.processNoise}
            onChange={(e) => onChange({ processNoise: Number(e.target.value) })}
            className="accent-indigo-400"
          />
        </label>

        <label className="flex flex-col gap-1 text-xs text-slate-400">
          <span className="flex justify-between">
            <span>Motion factor</span>
            <span className="font-mono text-slate-500">{config.motionNoise.toFixed(2)}</span>
          </span>
          <input
            type="range" min={0} max={0.5} step={0.01}
            value={config.motionNoise}
            onChange={(e) => onChange({ motionNoise: Number(e.target.value) })}
            className="accent-indigo-400"
          />
        </label>
      </div>

      {/* Sensor correction */}
      <div className="flex flex-col gap-2 border-t border-slate-800 pt-2">
        <div className="flex items-center justify-between">
          <span className="text-[11px] uppercase tracking-[0.14em] text-slate-600">Sensor correction</span>
          <button
            onClick={() => onChange({ sensorCorrection: !config.sensorCorrection })}
            className={`border px-2 py-0.5 text-xs transition-colors ${
              config.sensorCorrection
                ? 'border-indigo-500 text-indigo-400'
                : 'border-slate-700 text-slate-500 hover:border-slate-600 hover:text-slate-400'
            }`}
          >
            {config.sensorCorrection ? 'on' : 'off'}
          </button>
        </div>

        {config.sensorCorrection && (
          <label className="flex flex-col gap-1 text-xs text-slate-400">
            <span className="flex justify-between">
              <span>Correction rate</span>
              <span className="font-mono text-slate-500">{config.correctionRate.toFixed(1)} /s</span>
            </span>
            <input
              type="range" min={0.1} max={5} step={0.1}
              value={config.correctionRate}
              onChange={(e) => onChange({ correctionRate: Number(e.target.value) })}
              className="accent-indigo-400"
            />
          </label>
        )}
        <p className="text-xs text-slate-600">
          {config.sensorCorrection
            ? 'LiDAR hits correct drift — uncertainty shrinks when obstacles are visible.'
            : 'Correction off — uncertainty grows unchecked (open-loop dead reckoning).'}
        </p>
      </div>

      {/* Live stats */}
      <div className="flex flex-col gap-1 border-t border-slate-800 pt-2">
        <span className="text-[11px] uppercase tracking-[0.14em] text-slate-600">Live estimate</span>
        <span className="flex justify-between text-xs text-slate-400">
          <span>σ major</span>
          <span className="font-mono text-indigo-400">{stats.sigma1.toFixed(3)} m</span>
        </span>
        <span className="flex justify-between text-xs text-slate-400">
          <span>σ minor</span>
          <span className="font-mono text-indigo-400">{stats.sigma2.toFixed(3)} m</span>
        </span>
        <span className="flex justify-between text-xs text-slate-400">
          <span>Axis angle</span>
          <span className="font-mono text-slate-500">{angleDeg}°</span>
        </span>
        <span className="flex justify-between text-xs text-slate-400">
          <span>Correlation</span>
          <span className="font-mono text-slate-500">
            {(stats.sigma1 > 0 && stats.sigma2 > 0
              ? stats.P[1] / (stats.sigma1 * stats.sigma2)
              : 0
            ).toFixed(3)}
          </span>
        </span>
      </div>
    </div>
  )
}
