import type { LidarHit, SensorConfig } from '@/sim/sensors'

interface SensorPanelProps {
  config: SensorConfig
  onChange: (update: { lidar?: Partial<SensorConfig['lidar']> }) => void
  lidarHits: LidarHit[]
}

interface SliderSpec {
  key: keyof SensorConfig['lidar']
  label: string
  min: number
  max: number
  step: number
  format: (v: number) => string
}

const SLIDERS: SliderSpec[] = [
  { key: 'numRays',     label: 'Ray count',    min: 8,   max: 360, step: 4,    format: (v) => `${v}` },
  { key: 'range',       label: 'Max range',    min: 2,   max: 20,  step: 0.5,  format: (v) => `${v.toFixed(1)} m` },
  { key: 'rangeNoise',  label: 'Range noise',  min: 0,   max: 1,   step: 0.01, format: (v) => `${v.toFixed(2)} m` },
  { key: 'dropoutRate', label: 'Dropout rate', min: 0,   max: 0.5, step: 0.01, format: (v) => `${(v * 100).toFixed(0)}%` },
]

export const SensorPanel = ({ config, onChange, lidarHits }: SensorPanelProps) => {
  const { lidar } = config

  const validHits = lidarHits.filter((h) => h.hitObstacle && !h.dropped)
  const droppedCount = lidarHits.filter((h) => h.dropped).length
  const avgError =
    validHits.length > 0
      ? validHits.reduce((s, h) => s + Math.abs(h.measuredRange - h.trueRange), 0) / validHits.length
      : 0

  return (
    <div className="flex flex-col gap-3 p-3">
      {/* Mode toggle */}
      <div className="flex items-center justify-between">
        <span className="text-xs text-slate-400">Sensor mode</span>
        <button
          onClick={() => onChange({ lidar: { enabled: !lidar.enabled } })}
          className={`border px-2 py-0.5 text-xs transition-colors ${
            lidar.enabled
              ? 'border-lime-500 text-lime-400'
              : 'border-slate-700 text-slate-500 hover:border-slate-600 hover:text-slate-400'
          }`}
        >
          {lidar.enabled ? 'on' : 'off'}
        </button>
      </div>

      <p className="text-xs text-slate-600">
        {lidar.enabled
          ? 'Robot perceives obstacles through LiDAR — APF forces use the scanned hit cloud, not true positions.'
          : 'Perfect knowledge — robot uses true obstacle positions directly.'}
      </p>

      {/* LiDAR parameter sliders */}
      <div className="flex flex-col gap-2 border-t border-slate-800 pt-2">
        <span className="text-[11px] uppercase tracking-[0.14em] text-slate-600">LiDAR</span>
        {SLIDERS.map(({ key, label, min, max, step, format }) => {
          const value = lidar[key] as number
          return (
            <label key={key} className="flex flex-col gap-1 text-xs text-slate-400">
              <span className="flex justify-between">
                <span>{label}</span>
                <span className="font-mono text-slate-500">{format(value)}</span>
              </span>
              <input
                type="range"
                min={min}
                max={max}
                step={step}
                value={value}
                onChange={(e) => onChange({ lidar: { [key]: Number(e.target.value) } })}
                className="accent-lime-400"
              />
            </label>
          )
        })}
      </div>

      {/* Live scan stats */}
      {lidar.enabled && (
        <div className="flex flex-col gap-1 border-t border-slate-800 pt-2">
          <span className="text-[11px] uppercase tracking-[0.14em] text-slate-600">Live scan</span>
          {lidarHits.length === 0 ? (
            <p className="text-xs text-slate-700">Waiting for first scan…</p>
          ) : (
            <>
              <span className="flex justify-between text-xs text-slate-400">
                <span>Hits</span>
                <span className="font-mono text-slate-500">
                  {validHits.length} / {lidarHits.length}
                </span>
              </span>
              <span className="flex justify-between text-xs text-slate-400">
                <span>Dropped</span>
                <span className="font-mono text-slate-500">{droppedCount}</span>
              </span>
              {validHits.length > 0 && (
                <span className="flex justify-between text-xs text-slate-400">
                  <span>Avg range error</span>
                  <span className="font-mono text-slate-500">{avgError.toFixed(3)} m</span>
                </span>
              )}
            </>
          )}
        </div>
      )}
    </div>
  )
}
