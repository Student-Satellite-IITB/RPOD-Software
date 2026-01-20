## GroundMonitor (MJPEG + CSV logging)

`GroundMonitor` is a low-priority observer task that:
- drains the latest FeatureFrame / PoseEstimate / RNAVState (freshest-wins)
- optionally serves a MJPEG UI (snapshots copied from `VBNTask`)
- optionally logs CSV for offline analysis

### Enabling GroundMonitor

Create a `ground::GroundMonitorCtx` and pass it to `ground::TaskEntry`:

```cpp
ground::GroundMonitorCtx mon_ctx{};
mon_ctx.feat_in = &featureFrameQueue;
mon_ctx.pose_in = &poseEstimateQueue;
mon_ctx.state_in = nullptr;  // optional
mon_ctx.vbn = &vbn;                     // required for snapshots/server

mon_ctx.cfg.enable_server    = true;     // MJPEG HTTP server
mon_ctx.cfg.enable_snapshots = true;     // request copy from VBNTask
mon_ctx.cfg.enable_csv       = true;     // CSV logging
mon_ctx.cfg.out_dir          = "../tools/data/tmp/vbn_monitor";
mon_ctx.cfg.log_n            = 1000;     // number of frames logged
mon_ctx.cfg.log_every        = 1;        // log every N-th frame
mon_ctx.cfg.testcase         = ground::Test::CENTROID_LOG;
```

### CSV logging

CSV logging is controlled by:

- `enable_csv` (on/off)
- `testcase` (which CSV format to write)
- `log_n` (how many frames to log)
- `log_every` (subsample factor)

`logged` counts frames logged (not rows). Some testcases emit multiple CSV rows per frame.

CSV files are written under `cfg.out_dir`.

---
### Test: RANGE_LOG

File: `range_log.csv`

Header:
```
k,t_us,range_cm,reproj_rms_px
```

Rows:
- One row per logged pose update (`msg::PoseEstimate`).
- `k` is the logged frame index (0…`log_n-1`) — this is the logging counter, not the camera frame id.
- `t_us` is `PoseEstimate::t_exp_end_us` (exposure end timestamp in microseconds).
- `range_cm` is `PoseEstimate::range_m * 100`.
- `reproj_rms_px` is `PoseEstimate::reproj_rms_px`.

Logging cadence:
- `log_every = 1` logs every pose update (subject to “freshest-wins” queue draining).
- `log_every = N` logs every N-th iteration of the monitor’s internal counter.

---

### Test: CENTROID_LOG

File: `centroid_log.csv`

Header:
```
k,t_us,blob_i,u_px,v_px,area,intensity
```

Rows:
- One row per blob/feature per logged frame (multiple CSV rows per `k`).
- `k` is the logged frame index (0…`log_n-1`) — `logged` counts frames, not rows.
- `t_us` is `FeatureFrame::t_exp_end_us`.
- `blob_i` is the blob index within `FeatureFrame::feats[]` for that frame.
- `(u_px, v_px)` are centroid pixel coordinates.
- `area` is blob pixel area (number of pixels).
- `intensity` is blob intensity sum (units depend on upstream image DN scaling).

Important notes:
- `blob_i` is not a persistent identity across frames unless the detector guarantees stable ordering.
  For offline analysis (e.g., per-LED centroid variance), do association yourself
  (nearest-neighbor match to previous frame, or sorting by `(u,v)` with gating).
- The file typically contains `sum_k feat_count(k)` data rows.

Recommended usage for LED centroid characterization:
- Lock exposure + gain, LED current, geometry, threshold, and min-area.
- Increase `log_n` (e.g., 500–2000 frames) to get stable variance estimates.
- Gate out frames with blob merge/split using `area` / `intensity` statistics.

---

### Adding a new CSV testcase

1) Extend the testcase enum:
```cpp
enum class Test : uint8_t {
    NONE = 0,
    RANGE_LOG,
    CENTROID_LOG,
    MY_NEW_TEST,
};
```

2) Add a CSV open stanza (filename + header):
```cpp
if (ctx->cfg.enable_csv && ctx->cfg.testcase == Test::MY_NEW_TEST) {
    csv = open_csv(*ctx, "my_new_test.csv", "k,t_us,field1,field2\n");
}
```

3) Add the logging block in the main loop:
```cpp
else if (ctx->cfg.testcase == Test::MY_NEW_TEST) {
    if (/* got_new_* */ && (k % ctx->cfg.log_every == 0)) {
        // csv << logged << "," << ... << "\n";
        logged++;
        if (logged == ctx->cfg.log_n) {
            csv.flush();
            csv.close();
        }
    }
    k++;
}
```

Conventions:
- Prefer `logged` as the primary counter that runs 0…`log_n-1`.
- Include `t_us` (from the message timestamp) in every CSV so logs can be aligned across modules.
- If a testcase logs multiple rows per frame, keep `k` constant for all rows of that frame.

---

### MJPEG UI and snapshots

If `enable_server` is true:
- GroundMonitor starts an MJPEG HTTP server on `cfg.port`.
- Snapshots are automatically enabled (GroundMonitor forces `enable_snapshots = enable_server`).

Snapshot pipeline:
1) Periodically request a copy from `VBNTask` (`snapshot_period_ms`).
2) Drain copied frames, keep the newest.
3) Convert to 8-bit preview, annotate, JPEG-encode.
4) Push JPEG to MJPEG server (`stream_fps` caps output rate).

---

### RNAVFilter integration note (PoseEstimate vs StateEstimate)

If you are running `RNAVFilter` and feeding `GroundMonitorCtx::state_in` with `rnav::StateEstimateQueue`,
do not also pass `pose_in` unless you explicitly want both printed/logged.

Reason:
- The RNAVFilter thread often consumes (drains) the same `PoseEstimateQueue` produced by `VBNTask`.
- If both RNAVFilter and GroundMonitor attach to the same pose queue, the monitor can miss pose updates due to queue draining.

Recommended configuration:
- When using RNAVFilter:
  - `mon_ctx.state_in = &stateEstimateQueue;`
  - `mon_ctx.pose_in  = nullptr;`
- When not using RNAVFilter:
  - `mon_ctx.pose_in  = &poseEstimateQueue;`
  - `mon_ctx.state_in = nullptr;`

This avoids two consumers racing on the same queue and keeps the monitor view consistent.
