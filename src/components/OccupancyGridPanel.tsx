import type { OccupancyGridStats } from '@/sim/occupancyGrid'

interface OccupancyGridPanelProps {
  mappingEnabled: boolean
  onMappingToggle: (v: boolean) => void
  showOverlay: boolean
  onShowOverlayChange: (v: boolean) => void
  cellSize: number
  onCellSizeChange: (v: number) => void
  onClear: () => void
  stats: OccupancyGridStats
  sensorEnabled: boolean
}

export const OccupancyGridPanel = ({
  mappingEnabled,
  onMappingToggle,
  showOverlay,
  onShowOverlayChange,
  cellSize,
  onCellSizeChange,
  onClear,
  stats,
  sensorEnabled,
}: OccupancyGridPanelProps) => (
  <div className="flex flex-col gap-3 p-3">
    {/* Controls row */}
    <div className="flex items-center justify-between gap-2">
      <div className="flex items-center gap-2">
        <span className="text-xs text-slate-400">Mapping</span>
        <button
          onClick={() => onMappingToggle(!mappingEnabled)}
          className={`border px-2 py-0.5 text-xs transition-colors ${
            mappingEnabled
              ? 'border-sky-500 text-sky-400'
              : 'border-slate-700 text-slate-500 hover:border-slate-600 hover:text-slate-400'
          }`}
        >
          {mappingEnabled ? 'on' : 'off'}
        </button>
      </div>
      <div className="flex items-center gap-2">
        <span className="text-xs text-slate-400">Overlay</span>
        <button
          onClick={() => onShowOverlayChange(!showOverlay)}
          className={`border px-2 py-0.5 text-xs transition-colors ${
            showOverlay
              ? 'border-sky-500 text-sky-400'
              : 'border-slate-700 text-slate-500 hover:border-slate-600 hover:text-slate-400'
          }`}
        >
          {showOverlay ? 'on' : 'off'}
        </button>
      </div>
    </div>

    {mappingEnabled && !sensorEnabled && (
      <p className="text-xs text-amber-600">
        Enable LiDAR sensor mode (Sensors panel) to start mapping.
      </p>
    )}
    <p className="text-xs text-slate-600">
      {mappingEnabled && sensorEnabled
        ? 'Accumulating LiDAR scans into a Bayesian log-odds map. Dark = free · Amber = caution zone (inside Q*) · Red = occupied.'
        : 'Accumulates LiDAR snapshots (Phase 12) into a persistent probabilistic map. Requires sensor mode.'}
    </p>

    {/* Cell size slider */}
    <label className="flex flex-col gap-1 text-xs text-slate-400">
      <span className="flex justify-between">
        <span>Cell size</span>
        <span className="font-mono text-slate-500">{cellSize.toFixed(2)} m</span>
      </span>
      <input
        type="range"
        min={0.1}
        max={0.5}
        step={0.05}
        value={cellSize}
        onChange={(e) => onCellSizeChange(Number(e.target.value))}
        className="accent-sky-400"
      />
    </label>

    {/* Coverage stats */}
    <div className="flex flex-col gap-1 border-t border-slate-800 pt-2">
      <div className="flex items-center justify-between">
        <span className="text-[11px] uppercase tracking-[0.14em] text-slate-600">Map coverage</span>
        <button
          onClick={onClear}
          className="border border-slate-700 px-2 py-0.5 text-[11px] text-slate-500 transition-colors hover:border-slate-600 hover:text-slate-400"
        >
          Clear
        </button>
      </div>
      <span className="flex justify-between text-xs text-slate-400">
        <span>Coverage</span>
        <span className="font-mono text-slate-500">{stats.coveragePct.toFixed(1)}%</span>
      </span>
      <span className="flex justify-between text-xs text-slate-400">
        <span>Free cells</span>
        <span className="font-mono text-slate-500">{stats.free.toLocaleString()}</span>
      </span>
      <span className="flex justify-between text-xs text-slate-400">
        <span>Occupied cells</span>
        <span className="font-mono text-sky-600">{stats.occupied.toLocaleString()}</span>
      </span>
      <span className="flex justify-between text-xs text-slate-400">
        <span>Grid size</span>
        <span className="font-mono text-slate-500">
          {Math.round(Math.sqrt(stats.total))}×{Math.round(Math.sqrt(stats.total))}
        </span>
      </span>
    </div>
  </div>
)
