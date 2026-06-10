import { useState } from 'react'
import { SimulationCanvas } from '@/components/SimulationCanvas'
import { ControlPanel } from '@/components/ControlPanel'
import { ParameterPanel } from '@/components/ParameterPanel'
import { PlacementToolbar } from '@/components/PlacementToolbar'
import type { PlacementTool } from '@/components/PlacementToolbar'
import { OverlaysPanel } from '@/components/OverlaysPanel'
import { IntegrationPanel } from '@/components/IntegrationPanel'
import { LocalMinimaPanel } from '@/components/LocalMinimaPanel'
import { RecoveryPanel } from '@/components/RecoveryPanel'
import { MetricsPanel } from '@/components/MetricsPanel'
import { ExperimentPanel } from '@/components/ExperimentPanel'
import { BenchmarkPanel } from '@/components/BenchmarkPanel'
import { SensorPanel } from '@/components/SensorPanel'
import { OccupancyGridPanel } from '@/components/OccupancyGridPanel'
import { UncertaintyPanel } from '@/components/UncertaintyPanel'
import { SLAMPanel } from '@/components/SLAMPanel'
import { Panel } from '@/components/Panel'
import { useSimulationLoop } from '@/hooks/useSimulationLoop'
import type { Vector2 } from '@/lib/vector'

function App() {
  const sim = useSimulationLoop()
  const [tool, setTool] = useState<PlacementTool>('goal')
  const [showVectorField, setShowVectorField] = useState(false)
  const [fieldSpacing, setFieldSpacing] = useState(1)
  const [showHeatmap, setShowHeatmap] = useState(false)
  const [heatmapResolution, setHeatmapResolution] = useState(0.6)
  const [showOccupancyGrid, setShowOccupancyGrid] = useState(false)
  const [showUncertainty, setShowUncertainty] = useState(false)
  const [uncertaintySigmaLevel, setUncertaintySigmaLevel] = useState(2)
  const [showSLAM, setShowSLAM] = useState(false)
  const [showComparisonTraces, setShowComparisonTraces] = useState(false)

  const handleCanvasClick = (point: Vector2) => {
    if (tool === 'goal') sim.setGoal(point)
    else sim.toggleObstacle(point)
  }

  return (
    <div className="flex h-screen w-screen flex-col gap-3 bg-slate-950 p-3 text-slate-100">
      <header className="flex items-baseline gap-3 border-b border-slate-800 pb-3">
        <h1 className="text-base font-medium tracking-tight">Potential Field Navigation</h1>
        <span className="font-mono text-xs uppercase tracking-[0.18em] text-slate-600">
          phase 15 — SLAM lite
        </span>
      </header>

      <main className="flex min-h-0 flex-1 flex-col gap-3 lg:flex-row">
        <div className="min-h-0 min-w-0 flex-1">
          <SimulationCanvas
            state={sim.state}
            params={sim.params}
            showVectorField={showVectorField}
            fieldSpacing={fieldSpacing}
            showHeatmap={showHeatmap}
            heatmapResolution={heatmapResolution}
            comparisonTraces={sim.comparisonTraces}
            showComparisonTraces={showComparisonTraces}
            localMinima={sim.localMinima}
            recovery={sim.recovery}
            recoveryMethod={sim.recoveryMethod}
            algorithmComparison={sim.algorithmComparison}
            sensorConfig={sim.sensorConfig}
            lidarHits={sim.lidarHits}
            showOccupancyGrid={showOccupancyGrid}
            occupancyGrid={sim.occupancyGrid}
            showUncertainty={showUncertainty}
            uncertaintyStats={sim.uncertaintyStats}
            uncertaintySigmaLevel={uncertaintySigmaLevel}
            showSLAM={showSLAM}
            slamState={sim.slamState}
            onCanvasClick={handleCanvasClick}
          />
        </div>

        <aside className="flex w-full shrink-0 flex-col gap-1.5 overflow-y-auto lg:w-72">
          {/* ── Core controls (always visible) ── */}
          <Panel title="Placement">
            <PlacementToolbar tool={tool} onToolChange={setTool} />
          </Panel>

          <Panel title="Playback">
            <ControlPanel
              running={sim.running}
              timestep={sim.timestep}
              time={sim.state.time}
              stepCount={sim.state.stepCount}
              onTogglePlay={sim.togglePlay}
              onStep={sim.step}
              onReset={sim.reset}
              onTimestepChange={sim.setTimestep}
            />
          </Panel>

          <Panel title="Field Gains">
            <ParameterPanel params={sim.params} onChange={sim.setParams} />
          </Panel>

          {/* ── Sensor simulation ── */}
          <Panel title="Sensors" defaultOpen={false}>
            <SensorPanel
              config={sim.sensorConfig}
              onChange={sim.setSensorConfig}
              lidarHits={sim.lidarHits}
            />
          </Panel>

          {/* ── Occupancy grid mapping ── */}
          <Panel title="Occupancy Map" defaultOpen={false}>
            <OccupancyGridPanel
              mappingEnabled={sim.mappingEnabled}
              onMappingToggle={sim.setMappingEnabled}
              showOverlay={showOccupancyGrid}
              onShowOverlayChange={setShowOccupancyGrid}
              cellSize={sim.occupancyGridCellSize}
              onCellSizeChange={sim.setOccupancyGridCellSize}
              onClear={sim.clearOccupancyGrid}
              stats={sim.occupancyGridStats}
              sensorEnabled={sim.sensorConfig.lidar.enabled}
            />
          </Panel>

          {/* ── Uncertainty visualization ── */}
          <Panel title="Uncertainty" defaultOpen={false}>
            <UncertaintyPanel
              config={sim.uncertaintyConfig}
              onChange={sim.setUncertaintyConfig}
              stats={sim.uncertaintyStats}
              showEllipse={showUncertainty}
              onShowEllipseChange={setShowUncertainty}
              sigmaLevel={uncertaintySigmaLevel}
              onSigmaLevelChange={setUncertaintySigmaLevel}
            />
          </Panel>

          {/* ── SLAM Lite ── */}
          <Panel title="SLAM" defaultOpen={false}>
            <SLAMPanel
              slamEnabled={sim.slamEnabled}
              onSlamToggle={sim.setSlamEnabled}
              showOverlay={showSLAM}
              onShowOverlayChange={setShowSLAM}
              config={sim.slamConfig}
              onChange={sim.setSlamConfig}
              slamState={sim.slamState}
              sensorEnabled={sim.sensorConfig.lidar.enabled}
            />
          </Panel>

          {/* ── Algorithm comparison ── */}
          <Panel
            title="Algorithm Benchmark"
            headerRight={
              <button
                onClick={sim.replanAlgorithms}
                className="text-[11px] uppercase tracking-[0.12em] text-slate-500 hover:text-amber-400"
              >
                Replan
              </button>
            }
          >
            <BenchmarkPanel
              comparison={sim.algorithmComparison}
              apfLive={sim.liveMetrics}
              apfLastRun={sim.runHistory[sim.runHistory.length - 1] ?? null}
            />
          </Panel>

          {/* ── Overlays (collapsed by default) ── */}
          <Panel title="Overlays" defaultOpen={false}>
            <OverlaysPanel
              showVectorField={showVectorField}
              onVFChange={setShowVectorField}
              fieldSpacing={fieldSpacing}
              onSpacingChange={setFieldSpacing}
              showHeatmap={showHeatmap}
              onHeatmapChange={setShowHeatmap}
              heatmapResolution={heatmapResolution}
              onResolutionChange={setHeatmapResolution}
              showComparisonTraces={showComparisonTraces}
              onComparisonTracesChange={setShowComparisonTraces}
            />
          </Panel>

          {/* ── Analysis panels (collapsed by default) ── */}
          <Panel title="Integration Method" defaultOpen={false}>
            <IntegrationPanel
              method={sim.integrationMethod}
              onMethodChange={sim.setIntegrationMethod}
              activeRobot={sim.state.robot}
              comparisonTraces={sim.comparisonTraces}
            />
          </Panel>

          <Panel title="Local Minima" defaultOpen={false}>
            <LocalMinimaPanel diagnostics={sim.localMinima} />
          </Panel>

          <Panel title="Recovery" defaultOpen={false}>
            <RecoveryPanel
              method={sim.recoveryMethod}
              onMethodChange={sim.setRecoveryMethod}
              recovery={sim.recovery}
            />
          </Panel>

          {/* ── Research (collapsed by default) ── */}
          <Panel title="Metrics" defaultOpen={false}>
            <MetricsPanel
              live={sim.liveMetrics}
              history={sim.runHistory}
              onClearHistory={sim.clearHistory}
              onExportJSON={sim.exportRunsJSON}
              onExportCSV={sim.exportRunsCSV}
            />
          </Panel>

          <Panel title="Experiments" defaultOpen={false}>
            <ExperimentPanel
              savedConfigs={sim.savedConfigs}
              onSave={sim.saveConfig}
              onLoad={sim.loadConfig}
              onDelete={sim.deleteConfig}
            />
          </Panel>
        </aside>
      </main>
    </div>
  )
}

export default App
