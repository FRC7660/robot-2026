# Robot Slowdown / Stability Summary

## Scope
- Source: `/tmp/FRC_UserProgram.latest.log` (latest pull, 406 lines)
- Context: post-deploy checks for slowdown/crash and vision behavior

## Executive Summary
- The robot code did **not crash** in the latest run.
- The system shows repeated **20ms loop overruns**.
- The largest timing spike is in **vision pipeline startup** (`fetch` step), with a one-time ~250ms cycle.
- A **high-error accepted AprilTag fusion** (`err=4.951m`) appears in this run and likely causes pose discontinuity.

## Evidence

### 1) No crash/restart detected
- `********** Robot program starting **********` appears once.
- `robotInit start` appears once.
- No `Exception`, `FATAL`, `NoClassDefFoundError`, `NullPointerException`, or JVM termination markers found.

### 2) Loop overruns are present
- Repeated log entries:
  - `Loop time of 0.02s overrun`
  - `CommandScheduler loop overrun`
- Representative timing breakdown from overrun context:
  - `SwerveSubsystem.periodic(): ~0.012s`
  - `RunCommand.execute(): ~0.011s`
  - `robotPeriodic(): ~0.035s`
- Combined work exceeds the 20ms loop budget intermittently.

### 3) Vision pipeline startup spike
- Worst observed slow cycle:
  - `[12.355] [VisionPipeline] SLOW cycle timing=[fetch=214.6ms est=18.6ms select=16.7ms apply=0.1ms total=250.0ms]`
- Later vision timing averages are much lower, but startup spikes are severe.

### 4) High-error accepted vision update
- At `40.964s`:
  - `ACCEPT camera=BACK_CAMERA ts=40.885 tags=6 err=4.951 pose=(13.009, 0.654, -172.1deg)`
- Immediately followed by a visible fused-pose jump in telemetry.
- This suggests acceptance criteria currently allow a clearly bad measurement in at least one case.

## Likely Contributors
- Startup camera frame handling / unread-results backlog leading to large `fetch` time.
- Command scheduler load (`RunCommand.execute` + subsystem periodic + base robot periodic).
- Vision acceptance gate allowing at least one high-error estimate through.

## Suggested Next Debug/Action Items
1. Add a hard reject guard for very large fusion error (`err`) before applying vision measurement.
2. Cap/trim unread frame processing per cycle to bound startup `fetch`/`est` latency.
3. Add per-command timing instrumentation for `RunCommand.execute` payloads to identify heavy callbacks.
4. Keep SmartDashboard `Auto/Rejoin*` metrics enabled and correlate with accepted vision updates.

