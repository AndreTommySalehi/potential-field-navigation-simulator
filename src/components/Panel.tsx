import { useState, type ReactNode } from 'react'

interface PanelProps {
  title: string
  defaultOpen?: boolean
  /** Content rendered in the right side of the header (e.g. an action button). */
  headerRight?: ReactNode
  children: ReactNode
}

export const Panel = ({ title, defaultOpen = true, headerRight, children }: PanelProps) => {
  const [open, setOpen] = useState(defaultOpen)

  return (
    <section className="border border-slate-800 bg-slate-900">
      <button
        onClick={() => setOpen((v) => !v)}
        className="flex w-full items-center justify-between border-b border-slate-800 px-3 py-2 text-left transition-colors hover:bg-slate-800/30"
      >
        <span className="text-[11px] uppercase tracking-[0.18em] text-slate-500">{title}</span>
        <div className="flex items-center gap-2">
          {headerRight && (
            <div onClick={(e) => e.stopPropagation()}>
              {headerRight}
            </div>
          )}
          <svg
            className={`h-3 w-3 flex-shrink-0 text-slate-600 transition-transform duration-150 ${open ? '' : '-rotate-90'}`}
            fill="currentColor"
            viewBox="0 0 20 20"
          >
            <path
              fillRule="evenodd"
              d="M5.23 7.21a.75.75 0 011.06.02L10 11.168l3.71-3.938a.75.75 0 111.08 1.04l-4.25 4.5a.75.75 0 01-1.08 0l-4.25-4.5a.75.75 0 01.02-1.06z"
              clipRule="evenodd"
            />
          </svg>
        </div>
      </button>
      {open && children}
    </section>
  )
}
