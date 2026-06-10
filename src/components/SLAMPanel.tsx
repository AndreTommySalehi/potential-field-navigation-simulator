import type { SLAMConfig, SLAMState } from '@/sim/slam'

interface SLAMPanelProps {
  slamEnabled: boolean
  onSlamToggle: (v: boolean) => void
  showOverlay: boolean
  onShowOverlayChange: (v: boolean) => void
  config: SLAMConfig
  onChange: (update: Partial<SLAMConfig>) => void
  slamState: SLAMState
  sensorEnabled: boolean
}

export const SLAMPanel = ({
  slamEnabled,
  onSlamToggle,
  showOverlay,
  onShowOverlayChange,
  config,
  onChange,
  slamState,
  sensorEnabled,
}: SLAMPanelProps) => {
  const avgLandmarkSigma =
    slamState.landmarks.length > 0
      ? Math.sqrt(slamState.landmarks.reduce((s, l) => s + l.sigma2, 0) / slamState.landmarks.length)
      : 0

  return (
    <div className="flex flex-col gap-3 p-3">
      {/* Toggle row */}
      <div className="flex items-center justify-between gap-2">
        <div className="flex items-center gap-2">
          <span className="text-xs text-slate-400">SLAM</span>
          <button
            onClick={() => onSlamToggle(!slamEnabled)}
            className={`border px-2 py-0.5 text-xs transition-colors ${
              slamEnabled
                ? 'border-yellow-500 text-yellow-400'
                : 'border-slate-700 text-slate-500 hover:border-slate-600 hover:text-slate-400'
            }`}
          >
            {slamEnabled ? 'on' : 'off'}
          </button>
        </div>
        <div className="flex items-center gap-2">
          <span className="text-xs text-slate-400">Overlay</span>
          <button
            onClick={() => onShowOverlayChange(!showOverlay)}
            className={`border px-2 py-0.5 text-xs transition-colors ${
              showOverlay
                ? 'border-yellow-500 text-yellow-400'
                : 'border-slate-700 text-slate-500 hover:border-slate-600 hover:text-slate-400'
            }`}
          >
            {showOverlay ? 'on' : 'off'}
          </button>
        </div>
      </div>

      {slamEnabled && !sensorEnabled && (
        <p className="text-xs text-amber-600">
          Enable LiDAR sensor mode (Sensors panel) to run SLAM.
        </p>
      )}

      <p className="text-xs text-slate-600">
        Simultaneously estimates robot position AND obstacle landmark locations.
        Early observations build the map; established landmarks correct accumulated
        drift. A large correction = loop closure.
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
            type="range" min={0} max={0.02} step={0.001}
            value={config.processNoise}
            onChange={(e) => onChange({ processNoise: Number(e.target.value) })}
            className="accent-yellow-400"
          />
        </label>

        <label className="flex flex-col gap-1 text-xs text-slate-400">
          <span className="flex justify-between">
            <span>Motion factor</span>
            <span className="font-mono text-slate-500">{config.motionNoise.toFixed(2)}</span>
          </span>
          <input
            type="range" min={0} max={0.3} step={0.01}
            value={config.motionNoise}
            onChange={(e) => onChange({ motionNoise: Number(e.target.value) })}
            className="accent-yellow-400"
          />
        </label>

        <label className="flex flex-col gap-1 text-xs text-slate-400">
          <span className="flex justify-between">
            <span>Observation noise</span>
            <span className="font-mono text-slate-500">{config.observationNoise.toFixed(2)} m²</span>
          </span>
          <input
            type="range" min={0.01} max={0.5} step={0.01}
            value={config.observationNoise}
            onChange={(e) => onChange({ observationNoise: Number(e.target.value) })}
            className="accent-yellow-400"
          />
        </label>
      </div>

      {/* Data association */}
      <div className="flex flex-col gap-2 border-t border-slate-800 pt-2">
        <span className="text-[11px] uppercase tracking-[0.14em] text-slate-600">Association</span>

        <label className="flex flex-col gap-1 text-xs text-slate-400">
          <span className="flex justify-between">
            <span>Match threshold</span>
            <span className="font-mono text-slate-500">{config.dataAssocThreshold.toFixed(1)} m</span>
          </span>
          <input
            type="range" min={0.5} max={3.0} step={0.1}
            value={config.dataAssocThreshold}
            onChange={(e) => onChange({ dataAssocThreshold: Number(e.target.value) })}
            className="accent-yellow-400"
          />
        </label>

        <label className="flex flex-col gap-1 text-xs text-slate-400">
          <span className="flex justify-between">
            <span>Min obs to correct</span>
            <span className="font-mono text-slate-500">{config.minObsForCorrection}</span>
          </span>
          <input
            type="range" min={1} max={10} step={1}
            value={config.minObsForCorrection}
            onChange={(e) => onChange({ minObsForCorrection: Number(e.target.value) })}
            className="accent-yellow-400"
          />
        </label>
      </div>

      {/* Live stats */}
      <div className="flex flex-col gap-1 border-t border-slate-800 pt-2">
        <span className="text-[11px] uppercase tracking-[0.14em] text-slate-600">Live stats</span>

        <span className="flex justify-between text-xs text-slate-400">
          <span>Landmarks</span>
          <span className="font-mono text-yellow-400">{slamState.landmarks.length}</span>
        </span>
        <span className="flex justify-between text-xs text-slate-400">
          <span>Position error</span>
          <span className={`font-mono ${slamState.posError > 1 ? 'text-red-400' : 'text-yellow-400'}`}>
            {slamState.posError.toFixed(2)} m
          </span>
        </span>
        <span className="flex justify-between text-xs text-slate-400">
          <span>Loop closures</span>
          <span className="font-mono text-yellow-400">{slamState.closureCount}</span>
        </span>
        <span className="flex justify-between text-xs text-slate-400">
          <span>Avg landmark σ</span>
          <span className="font-mono text-slate-500">
            {avgLandmarkSigma > 0 ? `${avgLandmarkSigma.toFixed(3)} m` : '—'}
          </span>
        </span>
      </div>
    </div>
  )
}
