# Copilot Instructions

`AGENTS.md` is the primary source of truth for this repository.

- Support issue URLs and plain prompts.
- Use Graphify context first. If missing, set it up before broad repository inspection. Fall back only if setup fails, and report the exact failure.
- Read the task input first.
- Inspect the relevant code before editing.
- Use tests and validation for behavior changes.
- Avoid unrelated refactors.
- Verify actual output behavior, not just metadata or docs.
- Keep documentation synchronized with workflow and template changes.
- Use optional local context tools only when they are available and appropriate for the task.
