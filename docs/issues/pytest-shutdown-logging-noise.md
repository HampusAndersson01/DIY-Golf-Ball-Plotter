# Issue: Pytest Exit Emits Shutdown Logging Error

## Summary

After `pytest` completes, the backend shutdown hook still attempts to write a log line and produces a noisy logging exception:

```text
ValueError: I/O operation on closed file.
...
app/logging_setup.py:51
logging.getLogger("golf_ball_plotter").info("Backend shutdown complete")
```

This does not currently fail the test run, but it pollutes CI output and makes it harder to distinguish real failures from cleanup noise.

## Current Behavior

- Test suite finishes normally.
- Python logging emits an exception during shutdown.
- The message appears after pytest summary output.

## Expected Behavior

- Test runs exit cleanly with no post-summary logging tracebacks.
- Shutdown logging should either:
  - avoid writing after handlers/streams are closed, or
  - guard the final log call during interpreter teardown.

## Likely Area

- [app/logging_setup.py](C:/Users/hampe/Documents/GIT/Golfball%20Printer/app/logging_setup.py:51)

## Impact

- No known product/runtime regression.
- CI logs are noisier than necessary.
- Can hide real teardown issues in future runs.

## Reproduction

1. Run `pytest -q`
2. Wait for summary output
3. Observe the trailing logging traceback after the test summary

## Notes

- This is a backend cleanup issue, not a test failure.
- It is currently tolerated in CI, but should be fixed to keep required checks clean and easy to trust.
