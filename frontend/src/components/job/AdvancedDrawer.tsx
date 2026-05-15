import { useAppStore } from '../../store/appStore'
import type { DrawerTab } from '../../store/appStore'

const TABS: DrawerTab[] = ['summary', 'settings', 'gcode', 'logs', 'advanced']

type Props = {
  activeTab: DrawerTab
  onTab: (tab: DrawerTab) => void
}

export function AdvancedDrawer({ activeTab, onTab }: Props) {
  const settings = useAppStore((state) => state.settings)!
  const updateSetting = useAppStore((state) => state.updateSetting)
  const labels: Record<DrawerTab, string> = {
    summary: 'Summary',
    settings: 'Settings',
    gcode: 'G-code',
    logs: 'Logs',
    advanced: 'Advanced',
  }

  return (
    <section className="panel drawer-panel">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Drawer</div>
          <h2>Secondary Panels</h2>
        </div>
      </div>
      <div className="tab-row">
        {TABS.map((tab) => (
          <button key={tab} className={tab === activeTab ? 'active' : ''} onClick={() => onTab(tab)} type="button">
            {labels[tab]}
          </button>
        ))}
      </div>

      {activeTab === 'settings' ? (
        <div className="drawer-pane form-pane">
          <div className="field-grid compact two">
            <label>
              <span>Placement scale %</span>
              <input onChange={(event) => updateSetting('placementScale', Number(event.target.value))} type="number" value={settings.placementScale} />
            </label>
            <label>
              <span>Rotation deg</span>
              <input onChange={(event) => updateSetting('rotationDeg', Number(event.target.value))} type="number" value={settings.rotationDeg} />
            </label>
            <label>
              <span>Offset X</span>
              <input onChange={(event) => updateSetting('placementOffsetX', Number(event.target.value))} type="number" value={settings.placementOffsetX} />
            </label>
            <label>
              <span>Offset Y</span>
              <input onChange={(event) => updateSetting('placementOffsetY', Number(event.target.value))} type="number" value={settings.placementOffsetY} />
            </label>
            <label>
              <span>Infill density</span>
              <input onChange={(event) => updateSetting('infillDensity', Number(event.target.value))} type="number" value={settings.infillDensity} />
            </label>
            <label>
              <span>Infill angle</span>
              <input onChange={(event) => updateSetting('infillAngleDeg', Number(event.target.value))} type="number" value={settings.infillAngleDeg} />
            </label>
          </div>
        </div>
      ) : null}

      {activeTab === 'advanced' ? (
        <div className="drawer-pane form-pane">
          <div className="field-grid compact two">
            <label>
              <span>Color tolerance</span>
              <input onChange={(event) => updateSetting('colorTolerance', Number(event.target.value))} type="number" value={settings.colorTolerance} />
            </label>
            <label>
              <span>Min component px</span>
              <input onChange={(event) => updateSetting('minComponentAreaPx', Number(event.target.value))} type="number" value={settings.minComponentAreaPx} />
            </label>
            <label>
              <span>Region simplify px</span>
              <input onChange={(event) => updateSetting('regionSimplifyPx', Number(event.target.value))} type="number" value={settings.regionSimplifyPx} />
            </label>
            <label>
              <span>Sample step deg</span>
              <input onChange={(event) => updateSetting('sampleStepDeg', Number(event.target.value))} type="number" value={settings.sampleStepDeg} />
            </label>
          </div>
          <div className="toggle-grid">
            <label className="toggle">
              <input checked={settings.invertY} onChange={(event) => updateSetting('invertY', event.target.checked)} type="checkbox" />
              <span>Invert Y</span>
            </label>
            <label className="toggle">
              <input checked={settings.includeComments} onChange={(event) => updateSetting('includeComments', event.target.checked)} type="checkbox" />
              <span>Include comments</span>
            </label>
            <label className="toggle">
              <input checked={settings.outlineAfterFill} onChange={(event) => updateSetting('outlineAfterFill', event.target.checked)} type="checkbox" />
              <span>Outline after fill</span>
            </label>
            <label className="toggle">
              <input checked={settings.removeDuplicatePaths} onChange={(event) => updateSetting('removeDuplicatePaths', event.target.checked)} type="checkbox" />
              <span>Remove duplicate paths</span>
            </label>
          </div>
        </div>
      ) : null}
    </section>
  )
}
