# World Model Module

Owner: Person 2. Real `LocalMap`/`ObstacleSet` production from a live LiDAR point cloud, layered on the frozen contracts in [docs/INTERFACES.md](INTERFACES.md). `MockWorldModel` still exists and still runs — swapping in `real_world_model` is a launch-file change (`real_perception_planning_pipeline.launch.xml` runs it together with `real_planner`), matching the workflow in [docs/DEVELOPMENT.md](DEVELOPMENT.md).

## Status

**Unit-tested and buildable; not yet run against real FAST-LIO2 cloud output.** Same deliberate scoping as Planning: the ROS-free core (`VoxelMapper`, `clusterOccupied`, `ObstacleTracker`) was built and verified entirely with synthetic points — no Gazebo/PX4 required — and the contract tests drive `real_world_model` as a subprocess with a hand-built `PointCloud2`. Wiring it to FAST-LIO2's actual `/cloud_registered` is pending the Localization module's `/Odometry` verification on capable hardware (see docs/LOCALIZATION.md).

- `uav_world_model_core` (C++ library, no ROS dependency): `VoxelMapper` (rolling hit-count occupancy grid), `clusterOccupied` (26-connected component extraction), `ObstacleTracker` (frame-to-frame association, velocity, STATIC/DYNAMIC classification). gtest-covered.
- `real_world_model` (ROS 2 node): subscribes `cloud_topic` (default `/cloud_registered`) + `/localization/state`, publishes `/world_model/local_map` + `/world_model/obstacles` at `rate_hz` (default 5), same topics/QoS as `MockWorldModel`.
- `tests/contract/test_world_model_contracts.py`: subprocess-based, synthetic cloud, no simulator.

Not yet done:
- Never run against a real FAST-LIO2 registered cloud (density, noise, and rate all differ from the synthetic blobs in tests). Parameter defaults (`min_hits`, `decay_every_n_ticks`, `point_stride`, `min_cluster_voxels`) will need tuning against real data.

**Fixed:** tall/thin obstacles (a pole, a tree trunk, a pillar) used to become one bounding sphere sized by the object's full height, not its width — a 0.5m-radius, 2.5m-tall pillar produced a ~1.5m-radius sphere. After planner margins, that false width could close gaps that were actually flyable (this is exactly what made the visual demo's pillar field briefly unplannable — see the demo section below). `clusterOccupied` now slices each connected component into vertical layers no taller than `max_cluster_height_m` (default 1.2m) before spherizing each layer, so a tall obstacle becomes several stacked, width-accurate spheres instead of one over-wide one.
- No ray-based free-space carving: a voxel is freed only by hit-count decay, not by observing rays passing through it. Cheap and simple, but a fast-moving obstacle leaves evidence for up to `min_hits/decay` decay cycles (~4s at defaults) after vacating.
- Tracker is greedy nearest-neighbor with exponential velocity smoothing — no Kalman filter, no global assignment. Two obstacles crossing paths within the association gate (1m) can swap identities.

## Design

```
/cloud_registered (PointCloud2, map frame) ──┐
/localization/state ─────────────────────────┼─> real_world_model
                                             │      │
                                             v      v
                          VoxelMapper (rolling hit-count voxel window)
                                │                        │
                                v                        v
                     LocalMap (0/1 occupancy)   clusterOccupied → ObstacleTracker
                                                         │
                                                         v
                                                   ObstacleSet (ids, velocity, class)
```

Decisions that matter downstream:

- **Hit-count evidence, not single-shot marking.** A voxel becomes occupied only after `min_hits` (default 2) LiDAR returns land in it, saturating at `max_hits` (10); every `decay_every_n_ticks` ticks (10 ticks = 2s at 5Hz) all counts decay by 1. Sensor noise doesn't hallucinate walls; vacated space eventually frees itself; saturation bounds how long "eventually" is.
- **Chunked re-centering.** The window follows the vehicle but only re-centers when it strays > `recenter_threshold_m` (2m) from center, shifting by a whole number of voxels and block-copying surviving evidence. Rationale: every origin change forces `DStarLitePlanner` into a full re-`initialize()` (the rolling-map caveat in [docs/PLANNING.md](PLANNING.md)), so re-centering rarely and in chunks — instead of continuously — is what preserves the local planner's incremental property on the vast majority of ticks. This is the direct answer to that documented caveat.
- **Unknown space is published as free (0), never 255.** The LocalMap contract allows 255=unknown, but `Grid3D::loadOccupancy` treats any nonzero cell as an obstacle — publishing 255 would wall off all unexplored space and the vehicle could never leave its starting bubble. Optimistic navigation (unknown = traversable until observed otherwise) is the standard choice for local planning and matches what `MockWorldModel` already published. If Safety later wants unknown-awareness, that's a contract-level conversation, not a silent change here.
- **`map_valid` gates on localization, not on cloud data.** An *empty but correctly positioned* map is a usable answer ("nothing observed yet"); a map positioned on a guessed origin is not. So `map_valid` is true once a healthy `LocalizationState` has arrived, even before the first cloud.
- **Frame assumption (no TF).** The cloud on `cloud_topic` is assumed to already be in the frame we call "map" — FAST-LIO2's registered cloud is in its odom frame, which is the same frame `lio_state_bridge` passes through as `LocalizationState`. No TF lookup is performed. If a different cloud source is ever used, it must be pre-transformed.

## Parameters (all on `real_world_model`)

| Parameter | Default | Meaning |
|---|---|---|
| `resolution` | 0.2 | m/voxel (LocalMap contract default) |
| `size_x`/`size_y`/`size_z` | 50/50/30 | window = 10 x 10 x 6 m |
| `min_hits` / `max_hits` | 2 / 10 | occupancy threshold / saturation |
| `decay_per_call` / `decay_every_n_ticks` | 1 / 10 | evidence fade rate |
| `recenter_threshold_m` | 2.0 | window re-center trigger |
| `point_stride` | 2 | integrate every Nth cloud point |
| `min_cluster_voxels` | 3 | clusters smaller than this are noise |
| `max_cluster_height_m` | 1.2 | vertical slice height before spherizing (0 disables slicing) |
| `track_gate_m` | 1.0 | max association distance frame-to-frame |
| `dynamic_speed_mps` | 0.3 | smoothed speed above which CLASS_DYNAMIC |
| `max_missed_frames` | 3 | unmatched track lifetime |
| `cloud_topic` | `/cloud_registered` | point cloud source |
| `rate_hz` | 5.0 | publish rate (LocalMap contract: 5-10 Hz) |

## How to verify

Unit + lint + contract tests, no simulator:

```bash
cd ~/gps-denied-uav && colcon build --packages-select uav_world_model && colcon test --packages-select uav_world_model && colcon test-result --verbose
```

```bash
cd ~/gps-denied-uav && source install/setup.bash && pytest tests/contract/test_world_model_contracts.py -v
```

End-to-end with real Planning (still no Gazebo — MockLocalization provides the pose, so the map is empty and trajectories are straight lines; the point is that the real nodes interoperate):

```bash
ros2 launch uav_bringup real_perception_planning_pipeline.launch.xml
```

With real FAST-LIO2 cloud data and a live RViz view, on hardware that can run Gazebo (pending Localization's `/Odometry` verification — see [docs/LOCALIZATION.md](LOCALIZATION.md)):

```bash
ros2 launch uav_bringup full_real_pipeline.launch.xml
```

That's the real-Gazebo counterpart to `demo_mission.launch.xml` (which exists only because the primary dev machine can't run Gazebo — see its own header comment). Same RViz config, same `/viz/*` topics, but every number behind them comes from the real vehicle and real sensors instead of `demo_world`/`demo_flyer`.
