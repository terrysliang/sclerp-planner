# sclerp-planner
ScLERP-based motion planning stack with collision avoidance.

This repo is split into four modules:
- `core/`: Dual quaternions, POE kinematics, RMRC, and a local SE(3) ScLERP tracking planner (`planMotionSclerp`).
- `urdf/`: Loads a serial chain from URDF into the minimal `core::ManipulatorModel`.
- `collision/`: Wraps the core planner with FCL-based closest-contact queries and an LCP-based
  joint correction step (`planMotionSclerpWithCollision`).
- `gazebo/` (optional): Registers obstacles and exports a parseable scene file + Gazebo world,
  plus a simple joint-trajectory CSV playback plugin for Ignition Gazebo Sim 6.

## Build

This is a C++17 / CMake (>= 3.20) project.

Default build (`core` + `urdf`):

```bash
cmake -S . -B build
cmake --build build -j
```

Tip: if you only want the math/kinematics/planner pieces (no URDF parsing), build just `core`:

```bash
cmake -S . -B build -DSCLERP_BUILD_URDF=OFF
cmake --build build -j
```

Common toggles:
- `SCLERP_BUILD_CORE=ON|OFF`
- `SCLERP_BUILD_URDF=ON|OFF` (requires `urdfdom`)
- `SCLERP_BUILD_COLLISION=ON|OFF` (requires FCL + Assimp + Boost)
- `SCLERP_BUILD_GAZEBO=ON|OFF` (requires `SCLERP_BUILD_COLLISION=ON`, plus `sdformat12` + `urdfdom`)
- `SCLERP_BUILD_TESTS=ON|OFF`

Run tests (when enabled):

```bash
ctest --test-dir build --output-on-failure
```

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

## Collision-aware planning (optional)

The collision wrapper (`planMotionSclerpWithCollision`) uses FCL closest-contact queries plus an LCP-based
joint correction step. You provide:
- per-link STL meshes (base at index 0), plus optional per-link mesh offsets
- obstacle objects (boxes/spheres/cylinders/planes/meshes)

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

In code, use `sclerp::gazebo::ObstacleRegistry` to register obstacles (FCL objects) and export:
- `ObstacleRegistry::writeJson("scene.json")` exports a simple parseable JSON scene.
- `ObstacleRegistry::writeSdfWorld("world.sdf", opt)` writes an SDF world with the obstacles, plus either:
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

Note: `world.sdf` is not checked into this repo — it must be generated by your application (for example via `ObstacleRegistry::writeSdfWorld("world.sdf", opt)`).

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
sclerp::gazebo::ObstacleRegistry reg;
reg.loadFromSdfWorld("world_saved.sdf");  // imports static models (ignores "ground_plane" and "robot" by default)
```

## Conventions

Conventions used throughout:
- `Transform` is `Eigen::Isometry3d` (SE(3)), composed by left-multiplication.
- Twists/screw axes use ordering `[v; w]` (linear; angular) in the space/world frame.
- Units: meters and radians.
- `rotationDistance` is quaternion chordal distance (not angle in radians).

## References

- Anirban Sinha, Anik Sarker, Nilanjan Chakraborty, "Task Space Planning with Complementarity Constraint-based Obstacle Avoidance", arXiv:2104.07849v1 [cs.RO], 16 Apr 2021. https://arxiv.org/abs/2104.07849
