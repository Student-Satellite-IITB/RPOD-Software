## Testing Workflow
### How to Add New Tests Safely

This repo follows a strict rule:

> **Flight code stays clean.**  
- Tests live in `tests/`  
- Diagnostics / live-logging live in `tools/`

This avoids quick hacks creeping into the flight pipeline and keeps the codebase stable for integration.

---

### Architecture Overview

The runtime pipeline is organized as **tasks (threads)**:

1. **ImageCapture Task**  
   Reads frames from the camera device (e.g. `/dev/video0`) and publishes `msg::ImageFrame`.

2. **VBNTask**  
   Consumes `msg::ImageFrame`, runs the vision pipeline (FeatureDetector; optionally Pose Estimator), and publishes:
   - `msg::FeatureFrame`
   - `msg::PoseEstimate`

   VBNTask also supports **frame copy-outs** (for debugging/metrics) via:
   - `RequestCopy()`
   - `TryReceiveCopied()`
   - `ReleaseCopied()`

3. **GroundMonitor Task** (Observer only)  
   Low-priority task that:
   - drains queues (freshest-wins)
   - requests copied frames at a slow cadence
   - writes CSV logs / snapshots
   - optionally serves a MJPEG stream for live viewing

**Important:** GroundMonitor must never block or modify VBN behavior. It only observes.

---

### Directory Rules

Allowed:
- `tests/*.cpp`  
  Small *wiring* executables that create queues, configure modules, start tasks, and join tasks.
- `tools/groundmonitor/*`  
  Logging, CSV writing, MJPEG server, and derived metrics (saturation %, spot width proxies, etc.).
- `tools/data/tmp/*`  
  Output of tests (CSV, images).

Not allowed:
- Test-only fields inside `msg/*` types
- High-rate `std::cout` prints inside FD/VBN loops
- File I/O inside the VBN loop
- Random modifications to core pipeline modules for “just testing”

If you need extra test data, compute it in **GroundMonitor** using:
- `FeatureFrame` (centroids + area + IDs)
- `CopiedImage` (raw pixels from VBNTask copy-out)

---

### Practical Workflow (What You Actually Do)

#### Step 1 — Create a new test main in `tests/`
Copy an existing test (e.g. `vbn_pipeline_test.cpp`) and rename it if needed.

A test main should only do *wiring*:
- create queues
- configure modules
- instantiate task contexts
- start tasks (`ImageCapture`, `VBNTask`, `GroundMonitor`)
- join tasks

Avoid putting heavy logic in the test main. Tests should stay easy to read.

#### Step 2 — Decide what you want to measure
- If you only need LED locations/state → use `FeatureFrameQueue`
- If you need pixel-level data (saturation, blob width, resolvability, etc.) that is not exposed in `FeatureFrame` struct → use VBNTask copy-out:
  - `RequestCopy()`, `TryReceiveCopied()`, `ReleaseCopied()` 
and do the computation yourself on the copied image data in GroundMonitor.

#### Step 3 — Add helper functions inside `GroundMonitor.cpp` (recommended)
When adding new metrics or logs, do **not** modify FD/VBN code first.
Instead:

1) Add small helper functions in `GroundMonitor.cpp` under a clear comment header, e.g.

```cpp
// ------------------------------
// Custom Test Helpers (FD Metrics, etc.)
// ------------------------------
```

2) Use those helpers inside the existing while (true) loop in TaskEntry() at a slow cadence. This keeps flight code stable and puts all testing logic in one place.

#### Step 4 — Extend GroundMonitorConfig only when needed

If your new functionality needs optional switches or parameters (ROI size, logging cadence, output filename, etc.), add them to:

- `GroundMonitorConfig` in `GroundMonitor.hpp`

Keep them optional (default off). Typical pattern:

- `bool enable_xxx = false;`
- `uint32_t xxx_period_ms = 200;`
- `int xxx_roi_half = 12;`

Then in `TaskEntry()`:
only run the feature when the config flag is enabled
This prevents “always-on debug code” from affecting normal runs

#### Step 5 — Add the test executable in CMakeLists

After creating a new test file in `tests/` (e.g. `tests/fd_pipeline_test.cpp`), you must register it in the build system. Add an executable entry in the repo `../CMakeLists.txt`