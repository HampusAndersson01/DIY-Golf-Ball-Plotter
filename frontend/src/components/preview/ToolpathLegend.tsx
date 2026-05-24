const ITEMS = [
  { label: 'Walls / outlines', color: '#f6b756' },
  { label: 'Infill', color: '#46c0c6' },
  { label: 'Infill connector', color: '#84cc16' },
  { label: 'Detail trace', color: '#de5eb4' },
  { label: 'Travel', color: '#64748b' },
  { label: 'Active path', color: '#fff5a8' },
]

export function ToolpathLegend() {
  return (
    <div className="legend-row">
      {ITEMS.map((item) => (
        <span key={item.label}>
          <i style={{ background: item.color }} />
          {item.label}
        </span>
      ))}
    </div>
  )
}
