interface VectorFieldPanelProps {
  visible: boolean
  onVisibleChange: (visible: boolean) => void
  spacing: number
  onSpacingChange: (spacing: number) => void
}

const MIN_SPACING = 0.5
const MAX_SPACING = 3

export const VectorFieldPanel = ({ visible, onVisibleChange, spacing, onSpacingChange }: VectorFieldPanelProps) => (
  <section className="border border-slate-800 bg-slate-900">
    <h2 className="border-b border-slate-800 px-3 py-2 text-[11px] uppercase tracking-[0.18em] text-slate-500">
      Vector field
    </h2>
    <div className="flex flex-col gap-3 p-3">
      <button
        onClick={() => onVisibleChange(!visible)}
        className={`border px-3 py-1.5 text-left text-sm transition-colors ${
          visible
            ? 'border-teal-400 text-teal-300'
            : 'border-slate-700 text-slate-400 hover:border-slate-500 hover:text-slate-200'
        }`}
      >
        Overlay: {visible ? 'on' : 'off'}
      </button>

      <label className="flex flex-col gap-1 text-sm text-slate-400">
        <span className="flex justify-between">
          <span>Density</span>
          <span className="font-mono text-xs text-slate-500">{spacing.toFixed(1)} m spacing</span>
        </span>
        <input
          type="range"
          min={MIN_SPACING}
          max={MAX_SPACING}
          step={0.1}
          value={spacing}
          onChange={(e) => onSpacingChange(Number(e.target.value))}
          disabled={!visible}
          className="accent-teal-400 disabled:opacity-30"
        />
        <span className="text-xs text-slate-600">
          Each arrow samples the combined field at that point — direction shows where it pushes,
          length and brightness show how strongly.
        </span>
      </label>
    </div>
  </section>
)
