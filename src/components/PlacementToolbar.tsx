export type PlacementTool = 'goal' | 'obstacle'

interface PlacementToolbarProps {
  tool: PlacementTool
  onToolChange: (tool: PlacementTool) => void
}

const baseButton = 'flex-1 border px-3 py-1.5 text-sm transition-colors'
const activeGoal = 'border-emerald-400 text-emerald-300'
const activeObstacle = 'border-red-400 text-red-300'
const inactive = 'border-slate-700 text-slate-400 hover:border-slate-500 hover:text-slate-200'

export const PlacementToolbar = ({ tool, onToolChange }: PlacementToolbarProps) => (
  <div className="flex flex-col gap-2 p-3">
    <div className="flex gap-2">
      <button
        onClick={() => onToolChange('goal')}
        className={`${baseButton} ${tool === 'goal' ? activeGoal : inactive}`}
      >
        Goal
      </button>
      <button
        onClick={() => onToolChange('obstacle')}
        className={`${baseButton} ${tool === 'obstacle' ? activeObstacle : inactive}`}
      >
        Obstacle
      </button>
    </div>
    <p className="text-xs text-slate-500">
      {tool === 'goal'
        ? 'Click the field to move the goal.'
        : 'Click empty space to add · click again to remove.'}
    </p>
  </div>
)
