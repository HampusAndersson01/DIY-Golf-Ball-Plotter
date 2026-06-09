# Context Maintenance

Agents should keep repository instructions current instead of letting them drift from the actual workflow.

## What to keep in sync

- `AGENTS.md`
- `.github/copilot-instructions.md`
- `.github/ISSUE_TEMPLATE/*.yml`
- `.github/PULL_REQUEST_TEMPLATE.md`
- `docs/agent-workflows/*.md`
- `docs/ci-branch-protection.md` when CI checks change
- `README.md` when setup or validation commands change
- `graphify-out/` expectations and Graphify setup details when the local context workflow changes

## When `AGENTS.md` must change

Update `AGENTS.md` when any of these change:

- Setup commands
- Validation commands
- Branch naming rules
- Task-intake rules
- Geometry rules
- Machine/device prerequisites
- Local artifact handling guidance
- Graphify startup, refresh, or failure-handling guidance

## When workflow docs must change

Update the workflow docs when:

- The repository accepts a new task class
- The agent sequencing changes
- The active code path discovery process changes
- The PR handoff requirements change
- A repeated failure mode needs a documented recovery path
- Graphify commands, graph refresh rules, or graph output expectations change

## When templates must change

Update issue or PR templates when:

- The intake form needs a new required field
- A new evidence type is needed
- The repo needs a new issue class
- Reviewers need a more explicit checklist

## When device setup docs must change

Update device setup docs when:

- Python or Node version expectations change
- A new local dependency becomes required
- A tool becomes optional instead of required, or vice versa
- New local analysis tools need instructions
- Machine access steps change

## Preventing instruction drift

- Treat `AGENTS.md` as the canonical reference.
- Keep shorter docs as summaries or task-specific guides.
- Prefer updating the canonical file first, then align the supporting docs.
- Avoid duplicating long policy blocks in multiple places unless a short summary is enough.
- Re-read the docs before starting a task if the repository state has changed.
- Re-check Graphify setup and refresh rules if the local graph is stale or missing.

## Tasks that do not come from issues

Agents should not assume a GitHub issue exists.

If the task arrives as a prompt, bug report, screenshot, test failure, or other free-form input:

1. Classify the task type.
2. Identify the objective and constraints.
3. Note any missing acceptance criteria.
4. Inspect the repository before editing.
5. State assumptions explicitly if the prompt is incomplete.
6. Proceed with the smallest valid change.
