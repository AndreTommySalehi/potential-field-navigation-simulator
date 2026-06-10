interface HeatmapPanelProps {
  visible: boolean
  onVisibleChange: (visible: boolean) => void
  resolution: number
  onResolutionChange: (resolution: number) => void
}

const MIN_RESOLUTION = 0.4
const MAX_RESOLUTION = 2

export const HeatmapPanel = ({ visible, onVisibleChange, resolution, onResolutionChange }: HeatmapPanelProps) => (
  <section className="border border-slate-800 bg-slate-900">
    <h2 className="border-b border-slate-800 px-3 py-2 text-[11px] uppercase tracking-[0.18em] text-slate-500">
      Potential heatmap
    </h2>
    <div className="flex flex-col gap-3 p-3">
      <button
        onClick={() => onVisibleChange(!visible)}
        className={`border px-3 py-1.5 text-left text-sm transition-colors ${
          visible
            ? 'border-violet-400 text-violet-300'
            : 'border-slate-700 text-slate-400 hover:border-slate-500 hover:text-slate-200'
        }`}
      >
        Overlay: {visible ? 'on' : 'off'}
      </button>

      <label className="flex flex-col gap-1 text-sm text-slate-400">
        <span className="flex justify-between">
          <span>Resolution</span>
          <span className="font-mono text-xs text-slate-500">{resolution.toFixed(1)} m cells</span>
        </span>
        <input
          type="range"
          min={MIN_RESOLUTION}
          max={MAX_RESOLUTION}
          step={0.1}
          value={resolution}
          onChange={(e) => onResolutionChange(Number(e.target.value))}
          disabled={!visible}
          className="accent-violet-400 disabled:opacity-30"
        />
        <span className="text-xs text-slate-600">
          Dark = low potential (the goal sits in a well), bright = high (obstacle surfaces and
          everywhere far from the goal). Smaller cells look sharper but cost more to redraw.
        </span>
      </label>
    </div>
  </section>
)
