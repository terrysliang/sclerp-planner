# Python bindings (pybind11)

This project can optionally build a local Python extension module `sclerp` (pybind11) that exposes the APIs used by the C++ Gazebo examples.

## Build

From the repo root:

```bash
cmake -S . -B build \
  -DSCLERP_BUILD_PYTHON=ON \
  -DSCLERP_BUILD_CORE=ON \
  -DSCLERP_BUILD_URDF=ON \
  -DSCLERP_BUILD_TRAJECTORY=ON \
  -DSCLERP_BUILD_COLLISION=ON \
  -DSCLERP_BUILD_GAZEBO=ON
cmake --build build -j
```

The extension module is written to `build/python/`.

## Import

```bash
PYTHONPATH=build/python python3 -c "import sclerp; print('ok')"
```

## Run the Python demo scripts (ports of examples 1/2/3)

```bash
PYTHONPATH=build/python python3 python/scripts/example_1_no_obstacles_gazebo.py --out-dir sclerp_py_out
PYTHONPATH=build/python python3 python/scripts/example_2_hardcoded_obstacles_gazebo.py --out-dir sclerp_py_out
PYTHONPATH=build/python python3 python/scripts/example_3_scene_roundtrip_gazebo.py --out-dir sclerp_py_out
```

Each script writes:
- `world.sdf`
- `trajectory.csv`

## Gazebo plugin path

If you did not embed an absolute plugin filename into the exported `world.sdf`, you must add the build output path to `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` so Gazebo can load `libsclerp_joint_trajectory_player.so`:

```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="<sclerp_build>/gazebo:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"
ign gazebo "sclerp_py_out/world.sdf"
```

## Poses / transforms in Python

`core::Transform` is represented in Python as a 4×4 `float64` numpy matrix.

