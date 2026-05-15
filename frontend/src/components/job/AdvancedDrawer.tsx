import { useAppStore } from '../../store/appStore'
import type { DrawerTab } from '../../store/appStore'

const TABS: DrawerTab[] = ['advanced', 'gcode', 'logs']

type Props = {
  activeTab: DrawerTab
  onTab: (tab: DrawerTab) => void
}

export function AdvancedDrawer({ activeTab, onTab }: Props) {
  const settings = useAppStore((state) => state.settings)!
  const updateSetting = useAppStore((state) => state.updateSetting)
  const labels: Record<DrawerTab, string> = {
    advanced: 'Advanced',
    gcode: 'G-code',
    logs: 'Logs',
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

      {activeTab === 'advanced' ? (
        <div className="drawer-pane form-pane">
          <details className="details-panel">
            <summary>Placement</summary>
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
            </div>
          </details>

          <details className="details-panel">
            <summary>Toolpath</summary>
            <div className="field-grid compact two">
              <label>
                <span>Walls</span>
                <input onChange={(event) => updateSetting('wallCount', Number(event.target.value))} type="number" value={settings.wallCount} />
              </label>
              <label>
                <span>Infill density</span>
                <input onChange={(event) => updateSetting('infillDensity', Number(event.target.value))} type="number" value={settings.infillDensity} />
              </label>
              <label>
                <span>Infill spacing</span>
                <input onChange={(event) => updateSetting('infillSpacingMm', Number(event.target.value))} step="0.01" type="number" value={settings.infillSpacingMm} />
              </label>
              <label>
                <span>Infill angle</span>
                <input onChange={(event) => updateSetting('infillAngleDeg', Number(event.target.value))} type="number" value={settings.infillAngleDeg} />
              </label>
              <label>
                <span>Simplify mm</span>
                <input onChange={(event) => updateSetting('simplifyToleranceMm', Number(event.target.value))} step="0.01" type="number" value={settings.simplifyToleranceMm} />
              </label>
              <label>
                <span>Min stroke mm</span>
                <input onChange={(event) => updateSetting('minSegmentLengthMm', Number(event.target.value))} step="0.01" type="number" value={settings.minSegmentLengthMm} />
              </label>
            </div>
          </details>

          <details className="details-panel">
            <summary>Feeds</summary>
            <div className="field-grid compact two">
              <label>
                <span>Draw feed</span>
                <input onChange={(event) => updateSetting('drawFeed', Number(event.target.value))} type="number" value={settings.drawFeed} />
              </label>
              <label>
                <span>Travel feed</span>
                <input onChange={(event) => updateSetting('travelFeed', Number(event.target.value))} type="number" value={settings.travelFeed} />
              </label>
              <label>
                <span>Sample step deg</span>
                <input onChange={(event) => updateSetting('sampleStepDeg', Number(event.target.value))} type="number" value={settings.sampleStepDeg} />
              </label>
            </div>
          </details>

          <details className="details-panel">
            <summary>Image cleanup</summary>
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
                <span>Min region px</span>
                <input onChange={(event) => updateSetting('minRegionAreaPx', Number(event.target.value))} type="number" value={settings.minRegionAreaPx} />
              </label>
              <label>
                <span>Region simplify px</span>
                <input onChange={(event) => updateSetting('regionSimplifyPx', Number(event.target.value))} type="number" value={settings.regionSimplifyPx} />
              </label>
              <label>
                <span>Mask open px</span>
                <input onChange={(event) => updateSetting('maskOpenRadiusPx', Number(event.target.value))} type="number" value={settings.maskOpenRadiusPx} />
              </label>
              <label>
                <span>Mask close px</span>
                <input onChange={(event) => updateSetting('maskCloseRadiusPx', Number(event.target.value))} type="number" value={settings.maskCloseRadiusPx} />
              </label>
            </div>
          </details>

          <details className="details-panel">
            <summary>Options</summary>
            <div className="toggle-grid compact-toggles">
              <label className="toggle">
                <input checked={settings.invertY} onChange={(event) => updateSetting('invertY', event.target.checked)} type="checkbox" />
                <span>Invert Y</span>
              </label>
              <label className="toggle">
                <input checked={settings.includeComments} onChange={(event) => updateSetting('includeComments', event.target.checked)} type="checkbox" />
                <span>Comments</span>
              </label>
              <label className="toggle">
                <input checked={settings.outlineAfterFill} onChange={(event) => updateSetting('outlineAfterFill', event.target.checked)} type="checkbox" />
                <span>Outline after fill</span>
              </label>
              <label className="toggle">
                <input checked={settings.removeDuplicatePaths} onChange={(event) => updateSetting('removeDuplicatePaths', event.target.checked)} type="checkbox" />
                <span>Remove duplicates</span>
              </label>
              <label className="toggle">
                <input checked={settings.thinDetailMode} onChange={(event) => updateSetting('thinDetailMode', event.target.checked)} type="checkbox" />
                <span>Thin detail</span>
              </label>
              <label className="toggle">
                <input checked={settings.allowPenDownInfillConnectors} onChange={(event) => updateSetting('allowPenDownInfillConnectors', event.target.checked)} type="checkbox" />
                <span>Pen-down connectors</span>
              </label>
            </div>
          </details>
        </div>
      ) : null}
    </section>
  )
}
