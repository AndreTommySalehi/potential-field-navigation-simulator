import { useState } from 'react'
import type { ExperimentConfig } from '@/sim/metrics'

interface ExperimentPanelProps {
  savedConfigs: ExperimentConfig[]
  onSave: (name: string) => void
  onLoad: (config: ExperimentConfig) => void
  onDelete: (name: string) => void
}

export const ExperimentPanel = ({ savedConfigs, onSave, onLoad, onDelete }: ExperimentPanelProps) => {
  const [name, setName] = useState('')

  const handleSave = () => {
    const trimmed = name.trim()
    if (!trimmed) return
    onSave(trimmed)
    setName('')
  }

  return (
    <div className="flex flex-col gap-3 p-3">
      <div className="flex gap-2">
        <input
          type="text"
          value={name}
          onChange={(e) => setName(e.target.value)}
          onKeyDown={(e) => e.key === 'Enter' && handleSave()}
          placeholder="Config name…"
          className="min-w-0 flex-1 bg-slate-800 px-2 py-1.5 text-xs text-slate-200 placeholder-slate-600 outline-none focus:outline-amber-400/50"
        />
        <button
          onClick={handleSave}
          disabled={!name.trim()}
          className="bg-amber-400 px-3 py-1.5 text-xs font-medium text-slate-900 hover:bg-amber-300 disabled:cursor-not-allowed disabled:bg-slate-700 disabled:text-slate-500"
        >
          Save
        </button>
      </div>
      <p className="text-xs text-slate-600">
        Saves goal, obstacles, gains, integration method, and recovery method.
      </p>

      {savedConfigs.length === 0 ? (
        <p className="text-xs text-slate-600">No saved configs yet.</p>
      ) : (
        <div className="flex flex-col gap-1">
          <span className="text-[11px] uppercase tracking-[0.14em] text-slate-600">Saved</span>
          {savedConfigs.map((c) => (
            <div key={c.name} className="flex items-center gap-2 border border-slate-800 px-2 py-1">
              <span className="min-w-0 flex-1 truncate text-xs text-slate-300">{c.name}</span>
              <button
                onClick={() => onLoad(c)}
                className="shrink-0 text-xs text-amber-400 hover:text-amber-300"
              >
                Load
              </button>
              <button
                onClick={() => onDelete(c.name)}
                className="shrink-0 text-xs text-slate-600 hover:text-rose-400"
              >
                ×
              </button>
            </div>
          ))}
        </div>
      )}
    </div>
  )
}
