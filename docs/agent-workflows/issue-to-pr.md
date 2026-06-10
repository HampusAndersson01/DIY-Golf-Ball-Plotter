# Issue to PR

This workflow supports both issue-based work and prompt-based work.

## Supported starting points

- GitHub issue URL
- GitHub issue number
- Plain prompt
- Geometry bug report
- Documentation request
- CI or workflow request
- Test failure
- Code review request

## Workflow

1. Read task input.
2. Classify task type.
3. Ensure Graphify is installed and initialized.
4. Generate or refresh Graphify context.
5. Use Graphify to identify relevant files, active paths, and related docs or tests.
6. Sync repository state.
7. Create a task-scoped branch.
8. Inspect the targeted files directly.
9. Add or update tests when behavior changes.
10. Implement the smallest valid change.
11. Run validation.
12. Prepare the PR summary.

## Starting from a GitHub issue URL

1. Open the issue.
2. Extract the objective, acceptance criteria, and any referenced files.
3. Confirm whether the issue is about behavior, docs, workflow, or tooling.
4. Ensure Graphify is installed and the repository graph is up to date.
5. Use Graphify to find the active code path and the smallest relevant file set.
6. Create a branch with the issue number in the name when practical.
7. Work against the active code path, not just the file named in the issue.

Example branch names:

- `feature/issue-123-short-name`
- `fix/issue-123-short-name`

## Starting from a plain prompt

1. Treat the prompt as the source of truth.
2. State assumptions if the prompt is incomplete.
3. Ensure Graphify is installed and the repository graph is up to date.
4. Identify the affected areas with Graphify before editing.
5. Use a concise branch name that reflects the task.

Example branch names:

- `docs/agent-workflow-bootstrap`
- `workflow/ci-validation-update`
- `fix/geometry-outline-collapse`

## Starting from a geometry bug report

1. Trace the artifact through the active geometry pipeline.
2. Verify whether the failure is in preview, toolpath generation, or emitted G-code.
3. Use Graphify to confirm the most relevant geometry, preview, and export files.
4. Add a regression test against the live path.
5. Prove that the visible output changed, not just the metadata.

## Starting from a documentation-only prompt

1. Use Graphify to identify the docs and cross-references most likely to matter.
2. Identify the docs that need to change.
3. Keep the change minimal and aligned with the canonical instructions.
4. Update templates or workflow docs if the task changes future behavior.

## Starting from a CI or workflow prompt

1. Use Graphify to identify the workflow files and validation references.
2. Inspect the workflow file and the repository’s actual validation commands.
3. Match the documentation to the real CI jobs.
4. Avoid inventing extra checks that are not part of the repo.

## Pull request handoff

- Include the linked issue or prompt summary.
- Describe what changed and why.
- List validation commands and their outcomes.
- Mention any remaining risks or follow-up work.
- Keep the PR focused on the task, not unrelated cleanup.
