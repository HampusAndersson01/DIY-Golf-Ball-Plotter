# Branch Protection

To protect `main` with CI, configure a branch protection rule in GitHub for `main` and require these status checks:

- `Backend Tests`
- `Frontend Lint`
- `Frontend Build`

Recommended rule settings:

- Require a pull request before merging
- Require status checks to pass before merging
- Require branches to be up to date before merging
- Require conversation resolution before merging
- Do not allow force pushes
- Do not allow deletions

This repository now exposes the required checks through:

- [.github/workflows/ci.yml](C:/Users/hampe/Documents/GIT/Golfball%20Printer/.github/workflows/ci.yml:1)
