# Submaps, loop candidates, and geometric verification

This module connects frontend submaps, STD/Scan Context retrieval, Quatro/Nano-GICP registration, and the Pose3 loop backend. Both retrieval channels share submap IDs. Candidates for the same pair are merged, and a single loop constraint is submitted after geometric verification.

## Build and run

Use the build and replay entrypoints at the repository root. See the [project README](../../README.md) for dependency installation and environment variables. Run the following from the repository root:

```bash
bash scripts/build.sh
bash scripts/replay.sh --dataset car --bag /path/to/input.bag \
  --mode dual --out /path/to/new_results
```

`--mode` accepts `std_only`, `sc_only`, or `dual`. Use `--dataset dog` for the quadruped preset. The replay entrypoint manages node startup, message counts, queue draining, and result saving.

## Module entrypoints

| Entrypoint | Function |
|---|---|
| `scripts/live_pipeline.py` | Streaming submap archival, candidate merging, geometric verification, and backend acknowledgements |
| `scripts/pipeline.py archive` | Build submap archives from a backend input bag and scan manifest |
| `scripts/pipeline.py candidates` | Run retrieval on archived submaps and output candidates |
| `scripts/verify.py` | Geometrically verify candidates and output verified measurements |
| `sc_candidates` / `std_candidates` | Scan Context / STD retrieval executables |
| `registration` | Quatro coarse alignment and Nano-GICP refinement |

Retrieval results are marked `retrieved`; only independently verified results are marked `verified`. Candidates are not inserted directly into the pose graph.

## Data and coordinate conventions

- Submaps are identified by `session_id + submap_id`. Records include scan ranges, cloud hashes, raw anchor poses, and state timestamps.
- `raw_anchor_pose` is `T_W_B`, encoded as `[x,y,z,qx,qy,qz,qw]`.
- Local clouds satisfy `P_local = inverse(T_W_B) * P_world`. LiDAR extrinsics have already been applied by the frontend and must not be applied again.
- Scan association timestamps are used to pair messages; state timestamps come from `scan_manifest.csv`.
- SC yaw describes the current-to-historical rotation in gravity-aligned coordinates, not a full SE(3) initial estimate.
- SC uses `R_G_B = Rz(-yaw(T_W_B)) * R_W_B`. When the frontend world Z axis is used without independent validation, the gravity quality is marked accordingly. The geometry configuration controls whether this assumption is allowed.
- Loop IDs follow `session:loop:historical:current`; candidate records contain cloud hashes and evidence from each source.

## Retrieval and geometry parameters

Scan Context defaults to a 20×60 descriptor, an 80 m radius, and a 2 m height encoding offset. The offset is not the sensor mounting height. The streaming entrypoint provides `--gap`, `--seconds`, `--top-k`, and `--sc-threshold` for recent-submap exclusion, minimum time separation, candidate count, and distance threshold. STD is also subject to `skip_near_num` in its own YAML.

See [car_geometry.json](config/car_geometry.json) and [dog_geometry.json](config/dog_geometry.json) for:

- Downsampling size, minimum point count, and maximum correspondence distance.
- Bidirectional overlap, inlier distance RMSE, and point-to-plane RMSE.
- Spatial coverage, normalized Hessian eigenvalue ratio, and allowed changes from the initial estimate.
- Loop noise standard deviations and whether an independently unvalidated gravity assumption is allowed.

The STD initial estimate is converted to `T_Bh_Bc` using `inverse(X_h) * Delta_W * X_c`. SC constructs an initial estimate from yaw, gravity matrices, and relative translation from raw odometry. Quatro provides coarse alignment when needed, followed by Nano-GICP refinement. Successful hypotheses for the same submap pair are marked as conflicting if they differ by more than 1 m or 0.1 rad.

## Backend interface

The full pipeline uses:

```yaml
backend_mode: rebuildable
loop_input_mode: verified_pose3
loop_policy: native
loop_factors_en: true
adjacent_factors_en: false
frontend_feedback_en: false
max_loop_normalized_residual: 8.0
max_graph_normalized_residual: 20.0
```

`/loop_pipeline/verified_loops` publishes `std_msgs/String` messages containing JSON schema 1, including loop ID, submap pair, measurement, noise, and input hashes. The measurement convention is `T_Bh_Bc`. Noise order is `rx_ry_rz_tx_ty_tz`, with rotations in rad and translations in m.

Each submap pair contributes one `BetweenFactor<Pose3>` with a Cauchy(1) robust kernel. Identical retransmissions do not duplicate factors, and conflicting measurements do not replace the original. Trial optimization exceeding the residual thresholds restores the iSAM2 snapshot; rejected transactions are not written to the active factor archive. These are engineering thresholds, not calibrated false-positive probabilities.

`factor_transactions.jsonl` stores original measurements, noise, initial values, and factor indices. Use `rebuild_factor_archive` to reconstruct the graph in an independent iSAM2 instance. This does not provide online removal of previously accepted loops or checkpoint/resume.

## Shutdown and completeness

Drain upstream retrieval and verification first, then supply the backend with expected submap and verified-message counts and call the drain services. Final results are saved only after counts match and pending queues are empty. After an abnormal exit, inspect status and hashes before treating an archive as complete.

See [third-party notices](../../docs/THIRD_PARTY.md) for sources, pinned versions, and licenses.
