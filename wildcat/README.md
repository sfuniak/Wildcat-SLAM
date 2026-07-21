# Python bag processing

The `wildcat_slam` pybind11 extension keeps bag orchestration and ROS message
decoding in Python while running odometry in the existing C++ core.

```sh
uv sync
uv run process-bag recording.bag
```

`uv sync` creates and updates the project environment from `uv.lock`; no
manual virtual-environment activation or direct `pip` invocation is needed.
scikit-build-core configures CMake and builds the C++ pybind11 extension as
part of `uv sync`. Run `uv sync` again after changing the C++ sources.

To display the latest pose, trajectory, and sliding/fixed-window surfels in
Rerun while processing a bag:

```sh
uv sync --extra visualization
uv run --extra visualization process-bag recording.bag --visualize
```

Visualization is single-threaded: each completed sweep is copied into a
snapshot and logged before processing continues. Without `--visualize`, Rerun
is neither imported nor required.

The default topics match the original node. Use `--imu-topic` and
`--lidar-topic` for other bags. The point cloud must contain `x`, `y`, `z`, and
the field selected by `--point-time-field` (default: `timestamp`). `intensity`
and `ring` are optional. Point timestamps can be absolute or relative to the
message header; select the interpretation with `--point-time-mode`.

For custom orchestration, instantiate `wildcat_slam.Processor` and call
`add_imu(...)` and `add_lidar(...)` with NumPy arrays. The most recent estimate
is available as `processor.latest_pose`, and `processor.sweep_count` reports
how many sweeps have completed.
