# sclerp-planner
ScLERP-based motion planning stack with collision avoidance.

This repo is split into five modules:
- `core/`: Dual quaternions, POE kinematics, RMRC, and a local SE(3) ScLERP tracking planner (`planMotionSclerp`).
- `urdf/`: Loads a serial chain from URDF into the minimal `core::ManipulatorModel`.
- `trajectory/`: Time-parameterizes a joint-space path into an executable trajectory (samples of `t,q,qd,qdd`).
- `collision/`: Wraps the core planner with FCL-based closest-contact queries and an LCP-based
  joint correction step (`planMotionSclerpWithCollision`).
- `gazebo/` (optional): Registers obstacles and exports a parseable scene file + Gazebo world,
  plus a simple joint-trajectory CSV playback plugin for Ignition Gazebo Sim 6.

## Build

This is a C++17 / CMake (>= 3.20) project.

Default build (`core` + `urdf` + `trajectory`):

```bash
cmake -S . -B build
cmake --build build -j
```

Tip: if you only want the math/kinematics/planner pieces (no URDF parsing), disable `urdf`:

```bash
cmake -S . -B build -DSCLERP_BUILD_URDF=OFF
cmake --build build -j
```

Common toggles:
- `SCLERP_BUILD_CORE=ON|OFF`
- `SCLERP_BUILD_URDF=ON|OFF` (requires `urdfdom`)
- `SCLERP_BUILD_TRAJECTORY=ON|OFF`
- `SCLERP_BUILD_COLLISION=ON|OFF` (requires FCL + Assimp + Boost)
- `SCLERP_BUILD_GAZEBO=ON|OFF` (requires `SCLERP_BUILD_COLLISION=ON`, plus `sdformat12` + `urdfdom`)
- `SCLERP_BUILD_PYTHON=ON|OFF` (pybind11 extension; requires `SCLERP_BUILD_GAZEBO=ON` and thus `SCLERP_BUILD_COLLISION=ON`)
- `SCLERP_BUILD_TESTS=ON|OFF`
- `SCLERP_BUILD_EXAMPLES=ON|OFF` (requires `SCLERP_BUILD_GAZEBO=ON`)

Run tests (when enabled):

```bash
ctest --test-dir build --output-on-failure
```

## Examples (out-of-the-box)

The example executables under `examples/` bundle a small Diana7 robot asset set (URDF + simplified STL link meshes),
so you can run them without providing your own URDF/meshes.

Build:

```bash
cmake -S . -B build-examples -DSCLERP_BUILD_COLLISION=ON -DSCLERP_BUILD_GAZEBO=ON -DSCLERP_BUILD_EXAMPLES=ON
cmake --build build-examples -j
```

Run (no args; uses bundled assets by default):

```bash
./build-examples/examples/sclerp_example_1_no_obstacles_gazebo
./build-examples/examples/sclerp_example_2_hardcoded_obstacles_gazebo
./build-examples/examples/sclerp_example_3_scene_roundtrip_gazebo
```

Example 3 can also import a planning scene from an existing Gazebo world SDF (for example saved via the
`WorldSdfSaver` system plugin) and plan against the imported obstacles:

```bash
./build-examples/examples/sclerp_example_3_scene_roundtrip_gazebo --scene-world /abs/path/to/world_saved.sdf
```

Override the start joint configuration:

```bash
./build-examples/examples/sclerp_example_1_no_obstacles_gazebo \
  --q-init "1.88857, 0.167132, 0.0285466, 2.4741, -0.327457, -0.52395, 2.23218"
```

To use your own robot, pass `--urdf`, `--stl-dir`, `--base-link`, and `--tip-link` (all required if any are set).

## Minimal usage (core + urdf)

```cpp
#include <Eigen/Dense>

#include "sclerp/core/kinematics/kinematics_solver.hpp"
#include "sclerp/core/planning/motion_plan.hpp"
#include "sclerp/urdf/load_manipulator.hpp"

sclerp::urdf::LoadOptions uopt;
uopt.base_link = "base_link";
uopt.tip_link = "tool0";

const auto load = sclerp::urdf::loadManipulatorModelFromFile("/abs/path/to/robot.urdf", uopt);
if (!sclerp::core::ok(load.status)) {
  // load.message contains optional context.
}

sclerp::core::KinematicsSolver solver(load.model);

Eigen::VectorXd q0 = Eigen::VectorXd::Zero(solver.model().dof());
sclerp::core::Transform g0;
solver.forwardKinematics(q0, &g0);

sclerp::core::MotionPlanRequest req;
req.q_init = q0;
req.g_i = g0;  // sanity check (planner uses FK(q_init) internally)
req.g_f = g0;
req.g_f.translation().x() += 0.2;

const auto result = sclerp::core::planMotionSclerp(solver, req);
// result.status + result.path.positions
```

### Failure dumps (debug)

When enabled, `planMotionSclerp` / `planMotionSclerpWithCollision` will write a timestamped CSV joint-path dump (best effort) on non-success and print the dump path via the internal logger.

In C++ / Python, enable via planner options:
- `MotionPlanOptions.failure_log.enabled = true`
- `MotionPlanOptions.failure_log.dir = "/abs/or/relative/dir"` (optional; default: `./sclerp_failure_logs/`)

To visualize a dumped joint path in Gazebo (trajectory.csv + world.sdf):
- Python: `python/scripts/visualize_failure_dump_gazebo.py` (requires `SCLERP_BUILD_PYTHON=ON`, which builds and links the `trajectory` + `gazebo` modules).
- C++: `sclerp_gazebo_visualize_failure_dump --dump <dump.csv> --urdf ... --stl-dir ... --base-link ... --tip-link ...` (requires `SCLERP_BUILD_GAZEBO=ON`).

## Collision-aware planning (optional)

The collision wrapper (`planMotionSclerpWithCollision`) uses FCL closest-contact queries plus an LCP-based
joint correction step. You provide:
- per-link STL meshes (base at index 0), plus optional per-link mesh offsets
- obstacle objects (boxes/spheres/cylinders/planes/meshes)

`CollisionQueryOptions::use_obstacle_broadphase` is enabled by default. When enabled, obstacle-vs-link and
obstacle-vs-grasped-object environment queries are pruned through an internal FCL obstacle broadphase;
self-collision remains pairwise.

Broadphase reuse in `planMotionSclerpWithCollision` assumes the obstacle list and obstacle transforms stay fixed
for the duration of that planning call. Standalone `computeContacts(...)` also honors the option, but it may build
a temporary obstacle broadphase internally, so the main performance benefit comes from planner-level reuse.

## Trajectory generation

The core planner returns a `JointPath` (positions only). To make it executable on a controller (or to export a
time-stamped CSV), use the `trajectory` module to generate samples of `{t, q, qd, qdd}` under joint limits.

Built-in options are provided:
- `sclerp::trajectory::GridTotg` (grid-based forward/backward time parameterization)
- `sclerp::trajectory::planWithToppra` (optional TOPPRA backend; time-optimal parameterization with v/a limits)

Minimal pattern:

```cpp
#include "sclerp/trajectory/interpolator.hpp"

sclerp::trajectory::Limits lim;
lim.v_max = Eigen::VectorXd::Constant(solver.model().dof(), 1.0);
lim.a_max = Eigen::VectorXd::Constant(solver.model().dof(), 2.0);

const auto unwrap = sclerp::trajectory::continuousRevoluteMask(solver.model());

sclerp::trajectory::GridTotg gen(lim);
sclerp::trajectory::PlannedTrajectory traj;
gen.plan(result.path, /*sample_dt=*/0.01, &traj, unwrap);

sclerp::trajectory::writeTrajectoryCsv(traj, "traj.csv");
```

### TOPPRA backend (optional)

If you have TOPPRA installed, `trajectory` can optionally enable a TOPPRA-based time parameterization backend.
At configure time, CMake must be able to resolve `find_package(toppra)`. If TOPPRA isn't found, the project
still builds, but `planWithToppra(...)` will return `Status::Failure`.

Common configure patterns:

```bash
cmake -S . -B build -DSCLERP_TRAJECTORY_WITH_TOPPRA=ON -DCMAKE_PREFIX_PATH=/abs/path/to/toppra/prefix
```

or:

```bash
cmake -S . -B build -DSCLERP_TRAJECTORY_WITH_TOPPRA=ON -Dtoppra_DIR=/abs/path/to/toppra/lib/cmake/toppra
```

## Gazebo export (optional)

Enable the module:

```bash
cmake -S . -B build -DSCLERP_BUILD_COLLISION=ON -DSCLERP_BUILD_GAZEBO=ON
cmake --build build -j
```

If you only want the SDF/JSON export + CLI tool (and don’t have Gazebo Sim 6 dev packages installed), disable the
system plugin targets:

```bash
cmake -S . -B build -DSCLERP_BUILD_COLLISION=ON -DSCLERP_BUILD_GAZEBO=ON -DSCLERP_GAZEBO_BUILD_IGNITION_PLUGIN=OFF
cmake --build build -j
```

In code, use `sclerp::gazebo::WorldRegistry` (include `sclerp/gazebo/world_registry.hpp`) to register obstacles (FCL objects) and export:

- `WorldRegistry::writeJson("scene.json")` exports a simple parseable JSON scene.
- `WorldRegistry::writeSdfWorld("world.sdf", opt)` writes an SDF world with the obstacles, plus either:
  - `WorldExportOptions::robot_from_urdf` (inline robot generated from URDF + per-link STL meshes), or
  - `WorldExportOptions::robot` (include an existing Gazebo model / SDF).
- `sclerp::gazebo::writeJointTrajectoryCsv(path, dt, "trajectory.csv")` writes `time,<joint...>` CSV compatible with the plugin.

Minimal pattern (inline robot):

```cpp
sclerp::gazebo::WorldExportOptions opt;
sclerp::gazebo::RobotModelFromUrdf robot;
robot.name = "robot";
robot.urdf_path = "/abs/path/to/robot.urdf";
robot.base_link = "base_link";
robot.tip_link = "tool0";
robot.link_mesh_stl_files = {/* base, link1, ..., (optional tool) */};
robot.mesh_offset_transforms = {/* optional; same size as link_mesh_stl_files */};
opt.robot_from_urdf = robot;

sclerp::gazebo::JointTrajectoryPlayback jt;
jt.csv_path = "trajectory.csv";
opt.joint_trajectory = jt;
reg.writeSdfWorld("world.sdf", opt);
```

`robot.link_mesh_stl_files` aligns with the exported kinematic frames: index 0 is `base_link`, indices 1..dof are the movable joints in the URDF chain (base→tip order), and an optional last entry may be used for a tool frame (when the URDF has fixed joints after the last movable joint and `collapse_fixed_joints=true`, or when `tool_offset` is set).

If you already have a folder of per-link STLs, you can skip manually enumerating `link_mesh_stl_files` and instead set:

```cpp
robot.link_mesh_stl_directory = "/abs/path/to/stl_folder";  // contains base_link.stl, link1.stl, ...
```

The exporter matches STL basenames to URDF link names (case-insensitive), and also falls back to URDF link mesh filenames when present.

To generate a robot-only `world.sdf` without writing any C++ code, build the module and run:

```bash
./build/gazebo/sclerp_gazebo_generate_world \
  --urdf /abs/path/to/robot.urdf \
  --stl-dir /abs/path/to/stl_folder \
  --base-link base_link \
  --tip-link tool0 \
  --out world.sdf
```

If the Ignition plugin target builds (`libsclerp_joint_trajectory_player.so`), make it discoverable by Gazebo Sim 6, e.g.:

```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$PWD/build/gazebo:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"
ign gazebo "$PWD/world.sdf"
```

Note: `world.sdf` is not checked into this repo — it must be generated by your application (for example via `WorldRegistry::writeSdfWorld("world.sdf", opt)`).

If `ign gazebo` prints `Unable to find or download file`, it typically means the world file path (or a referenced mesh URI) can’t be resolved. Use an absolute path for the world file (as above) and absolute STL paths (the exporter will write `file://...` URIs for absolute paths).

Note: when `opt.joint_trajectory`, `opt.world_sdf_saver`, or `opt.primitive_resizer` is set, the exported world includes Gazebo’s default systems (`Physics`, `UserCommands`, `SceneBroadcaster`), because Gazebo Sim won’t auto-load them when a custom system plugin is present.

Note: `RobotModelFromUrdf` exports `<static>true</static>` by default (fixed-base) so the robot won’t fall under gravity during kinematic playback. Set `robot.static_model = false` if you want a dynamic model.

To visualize the TCP path in the 3D scene, add the built-in Gazebo GUI plugin `Plot3D` and set `entity_name` to your TCP link’s scoped name (for example `robot::tool0`). It will draw a trail/curve as the link moves; tune `maximum_points` / `minimum_distance` to control density.

### Resize primitive obstacles

Enable the primitive-resizer system plugin in the exported world:

```cpp
opt.primitive_resizer = sclerp::gazebo::PrimitiveResizerOptions{};
```

Then call the service (this deletes + respawns the model with the new geometry, preserving pose):

```bash
ign service -s /world/sclerp_world/sclerp/resize_primitive \
  --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean \
  --timeout 5000 --req 'data: "box obstacle_box 0.5 0.4 0.3"'
```

Supported commands:
- `box <model_name> <x> <y> <z>`
- `sphere <model_name> <radius>`
- `cylinder <model_name> <radius> <length>`

### GUI obstacle save to SDF

Gazebo’s GUI already lets you move them around. To export the edited scene back into a planner-readable `world.sdf`, enable the `WorldSdfSaver` system plugin in the exported world:

```cpp
sclerp::gazebo::WorldSdfSaverOptions saver;
saver.output_path = "world_saved.sdf";
opt.world_sdf_saver = saver;
```

Run Gazebo, edit the scene in the GUI, then save:

```bash
ign service -s /world/sclerp_world/sclerp/save_world_sdf \
  --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean \
  --timeout 5000 --req 'data: "world_saved.sdf"'
```

Then load obstacles back into the planner:

```cpp
sclerp::gazebo::WorldRegistry reg;
reg.loadFromSdfWorld("world_saved.sdf");  // imports static models (ignores "ground_plane" and "robot" by default)
```

## Examples

This repo includes small end-to-end examples that:
1) plan (with / without obstacles),
2) time-parameterize into a sampled trajectory,
3) export `world.sdf` + `trajectory.csv` for Gazebo Sim 6 playback.

Build:

```bash
cmake -S . -B build -DSCLERP_BUILD_COLLISION=ON -DSCLERP_BUILD_GAZEBO=ON -DSCLERP_BUILD_EXAMPLES=ON
cmake --build build -j
```

Executables:
- `./build/examples/sclerp_example_1_no_obstacles_gazebo`
- `./build/examples/sclerp_example_2_hardcoded_obstacles_gazebo`
- `./build/examples/sclerp_example_3_scene_roundtrip_gazebo`

## Conventions

Conventions used throughout:
- `Transform` is `Eigen::Isometry3d` (SE(3)), composed by left-multiplication.
- Twists/screw axes use ordering `[v; w]` (linear; angular) in the space/world frame.
- Units: meters and radians.
- `rotationDistance` is quaternion chordal distance (not angle in radians).

## References

- Anirban Sinha, Anik Sarker, Nilanjan Chakraborty, "Task Space Planning with Complementarity Constraint-based Obstacle Avoidance", arXiv:2104.07849v1 [cs.RO], 16 Apr 2021. https://arxiv.org/abs/2104.07849
