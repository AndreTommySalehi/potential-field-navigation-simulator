interface OverlaysPanelProps {
  showVectorField: boolean
  onVFChange: (v: boolean) => void
  fieldSpacing: number
  onSpacingChange: (v: number) => void
  showHeatmap: boolean
  onHeatmapChange: (v: boolean) => void
  heatmapResolution: number
  onResolutionChange: (v: number) => void
  showComparisonTraces: boolean
  onComparisonTracesChange: (v: boolean) => void
}

export const OverlaysPanel = ({
  showVectorField,
  onVFChange,
  fieldSpacing,
  onSpacingChange,
  showHeatmap,
  onHeatmapChange,
  heatmapResolution,
  onResolutionChange,
  showComparisonTraces,
  onComparisonTracesChange,
}: OverlaysPanelProps) => (
  <div className="flex flex-col">
    {/* Vector field */}
    <div className="flex flex-col gap-2 border-b border-slate-800 p-3">
      <div className="flex items-center justify-between">
        <span className="text-xs text-slate-400">
          Vector field <span className="text-teal-500">·</span>
        </span>
        <button
          onClick={() => onVFChange(!showVectorField)}
          className={`border px-2 py-0.5 text-xs transition-colors ${
            showVectorField
              ? 'border-teal-500 text-teal-400'
              : 'border-slate-700 text-slate-500 hover:border-slate-600 hover:text-slate-400'
          }`}
        >
          {showVectorField ? 'on' : 'off'}
        </button>
      </div>
      {showVectorField && (
        <label className="flex flex-col gap-1 text-xs text-slate-400">
          <span className="flex justify-between">
            <span>Spacing</span>
            <span className="font-mono text-slate-500">{fieldSpacing.toFixed(1)} m</span>
          </span>
          <input
            type="range"
            min={0.5}
            max={3}
            step={0.1}
            value={fieldSpacing}
            onChange={(e) => onSpacingChange(Number(e.target.value))}
            className="accent-teal-400"
          />
        </label>
      )}
    </div>

    {/* Potential heatmap */}
    <div className="flex flex-col gap-2 p-3">
      <div className="flex items-center justify-between">
        <span className="text-xs text-slate-400">
          Heatmap <span className="text-violet-400">·</span>
        </span>
        <button
          onClick={() => onHeatmapChange(!showHeatmap)}
          className={`border px-2 py-0.5 text-xs transition-colors ${
            showHeatmap
              ? 'border-violet-500 text-violet-400'
              : 'border-slate-700 text-slate-500 hover:border-slate-600 hover:text-slate-400'
          }`}
        >
          {showHeatmap ? 'on' : 'off'}
        </button>
      </div>
      {showHeatmap && (
        <label className="flex flex-col gap-1 text-xs text-slate-400">
          <span className="flex justify-between">
            <span>Resolution</span>
            <span className="font-mono text-slate-500">{heatmapResolution.toFixed(1)} m</span>
          </span>
          <input
            type="range"
            min={0.4}
            max={2}
            step={0.1}
            value={heatmapResolution}
            onChange={(e) => onResolutionChange(Number(e.target.value))}
            className="accent-violet-400"
          />
        </label>
      )}
    </div>

    {/* Integration method ghosts */}
    <div className="flex flex-col gap-2 border-t border-slate-800 p-3">
      <div className="flex items-center justify-between">
        <span className="text-xs text-slate-400">
          Method ghosts <span className="text-orange-400">·</span>
        </span>
        <button
          onClick={() => onComparisonTracesChange(!showComparisonTraces)}
          className={`border px-2 py-0.5 text-xs transition-colors ${
            showComparisonTraces
              ? 'border-orange-500 text-orange-400'
              : 'border-slate-700 text-slate-500 hover:border-slate-600 hover:text-slate-400'
          }`}
        >
          {showComparisonTraces ? 'on' : 'off'}
        </button>
      </div>
      {showComparisonTraces && (
        <p className="text-xs text-slate-600">
          Euler / SIE / RK4 ghost robots run in parallel. Enable the Integration Method panel to compare their trajectories.
        </p>
      )}
    </div>
  </div>
)
