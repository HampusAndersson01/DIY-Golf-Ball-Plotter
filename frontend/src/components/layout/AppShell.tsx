import type { ReactNode } from 'react'

type Props = {
  topBar: ReactNode
  stepNav: ReactNode
  leftRail: ReactNode
  workspace: ReactNode
  rightRail: ReactNode
}

export function AppShell({ topBar, stepNav, leftRail, workspace, rightRail }: Props) {
  return (
    <div className="app-shell">
      {topBar}
      {stepNav}
      <div className="dashboard-grid">
        <aside className="left-rail">{leftRail}</aside>
        <main className="workspace-panel">{workspace}</main>
        <aside className="right-rail">{rightRail}</aside>
      </div>
    </div>
  )
}
