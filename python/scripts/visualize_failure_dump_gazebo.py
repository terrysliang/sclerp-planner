#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional

import numpy as np

import sclerp

from example_common import print_gazebo_hint


REPO_ROOT = Path(__file__).resolve().parents[2]


def _load_failure_dump_joint_path(csv_path: Path) -> sclerp.core.JointPath:
    if not csv_path.is_file():
        raise RuntimeError(f"Failure dump not found: {csv_path}")

    header: Optional[list[str]] = None
    rows: list[np.ndarray] = []

    with csv_path.open("r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if header is None:
                header = [t.strip() for t in line.split(",")]
                if any(not t for t in header):
                    raise RuntimeError("Invalid header (empty joint name)")
                continue

            toks = [t.strip() for t in line.split(",")]
            if len(toks) != len(header):
                raise RuntimeError(
                    f"Invalid waypoint row: expected {len(header)} values, got {len(toks)}"
                )
            rows.append(np.array([float(t) for t in toks], dtype=np.float64))

    if header is None:
        raise RuntimeError("Failure dump CSV has no header")
    if not rows:
        raise RuntimeError("Failure dump CSV has no waypoints")

    path = sclerp.core.JointPath()
    path.joint_names = header
    path.positions = rows
    return path


def main() -> int:
    p = argparse.ArgumentParser(
        description="Load a planner failure dump (joint-path CSV) and export trajectory.csv + world.sdf for Gazebo playback."
    )
    p.add_argument("--dump", type=str, required=True, help="Path to failure dump CSV (from MotionPlanOptions.failure_log)")
    p.add_argument("--out-dir", type=str, default=None, help="Output directory (default: <dump_dir>/<dump_stem>_viz)")

    p.add_argument("--urdf", type=str, default=None, help="Robot URDF path (default: bundled diana7)")
    p.add_argument("--stl-dir", type=str, default=None, help="Directory with per-link STL meshes (default: bundled diana7)")
    p.add_argument("--base-link", type=str, default=None, help="Base link name (default: base_link)")
    p.add_argument("--tip-link", type=str, default=None, help="Tip link name (default: link7)")
    p.add_argument("--no-collapse-fixed-joints", action="store_true", help="Do not collapse fixed joints in the URDF chain")

    p.add_argument("--scene-world", type=str, default=None, help="Optional SDF world to import obstacles from")
    p.add_argument("--world-name", type=str, default="sclerp_world")
    p.add_argument("--plugin-dir", type=str, default=None, help="Directory containing libsclerp_joint_trajectory_player.so")

    p.add_argument("--sample-dt", type=float, default=0.01, help="Sampling dt for TOTG and exported CSV")
    p.add_argument("--vmax", type=float, default=1.0, help="Max joint velocity for TOTG")
    p.add_argument("--amax", type=float, default=2.0, help="Max joint acceleration for TOTG")
    p.add_argument("--no-totg", action="store_true", help="Skip TOTG; export CSV by assigning uniform time steps")
    p.add_argument("--rate", type=float, default=1.0, help="Gazebo playback rate multiplier")
    p.add_argument("--loop", action="store_true", help="Loop playback")

    ns = p.parse_args()

    dump_path = Path(ns.dump).expanduser().resolve()
    out_dir = (
        Path(ns.out_dir).expanduser().resolve()
        if ns.out_dir
        else (dump_path.parent / f"{dump_path.stem}_viz").resolve()
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    bundled_urdf = REPO_ROOT / "examples" / "assets" / "diana7" / "urdf" / "diana_robot.urdf"
    bundled_stl_dir = REPO_ROOT / "examples" / "assets" / "diana7" / "meshes"

    urdf_path = Path(ns.urdf).expanduser().resolve() if ns.urdf else bundled_urdf.resolve()
    stl_dir = Path(ns.stl_dir).expanduser().resolve() if ns.stl_dir else bundled_stl_dir.resolve()
    base_link = str(ns.base_link) if ns.base_link else "base_link"
    tip_link = str(ns.tip_link) if ns.tip_link else "link7"
    collapse_fixed_joints = not bool(ns.no_collapse_fixed_joints)

    path = _load_failure_dump_joint_path(dump_path)

    uopt = sclerp.urdf.LoadOptions()
    uopt.base_link = base_link
    uopt.tip_link = tip_link
    uopt.collapse_fixed_joints = collapse_fixed_joints
    uopt.strict = True

    load = sclerp.urdf.load_manipulator_model_from_file(str(urdf_path), uopt)
    if not sclerp.core.ok(load.status):
        raise RuntimeError(f"load_manipulator_model_from_file failed: {load.message}")

    solver = sclerp.core.KinematicsSolver(load.model)
    dof = solver.model().dof()

    if len(path.joint_names) != dof:
        raise RuntimeError(f"Failure dump DOF ({len(path.joint_names)}) does not match URDF DOF ({dof})")
    for q in path.positions:
        if np.asarray(q).shape != (dof,):
            raise RuntimeError("Failure dump has inconsistent waypoint sizes")

    csv_path = out_dir / "trajectory.csv"

    if bool(ns.no_totg) or len(path.positions) < 2:
        st_csv = sclerp.gazebo.write_joint_trajectory_csv(path, float(ns.sample_dt), str(csv_path))
        if not sclerp.core.ok(st_csv):
            raise RuntimeError(f"write_joint_trajectory_csv failed: status={st_csv}")
    else:
        lim = sclerp.trajectory.Limits()
        lim.v_max = np.full((dof,), float(ns.vmax), dtype=np.float64)
        lim.a_max = np.full((dof,), float(ns.amax), dtype=np.float64)

        cfg = sclerp.trajectory.GridTotg.Config()
        cfg.unwrap_angles = True

        totg = sclerp.trajectory.GridTotg(lim, cfg)
        unwrap_mask = sclerp.trajectory.continuous_revolute_mask(solver.model())
        st_totg, traj = totg.plan(path, float(ns.sample_dt), unwrap_mask)
        if not sclerp.core.ok(st_totg):
            print(f"[visualize_failure_dump_gazebo.py] WARN: GridTotg.plan failed ({st_totg}); falling back to uniform dt", file=sys.stderr)
            st_csv = sclerp.gazebo.write_joint_trajectory_csv(path, float(ns.sample_dt), str(csv_path))
            if not sclerp.core.ok(st_csv):
                raise RuntimeError(f"write_joint_trajectory_csv failed: status={st_csv}")
        else:
            st_csv = sclerp.trajectory.write_trajectory_csv(traj, str(csv_path), sclerp.trajectory.CsvMode.PositionOnly)
            if not sclerp.core.ok(st_csv):
                raise RuntimeError(f"write_trajectory_csv failed: status={st_csv}")

    reg = sclerp.gazebo.WorldRegistry()
    if ns.scene_world:
        scene_world_path = Path(ns.scene_world).expanduser().resolve()
        st_load = reg.load_from_sdf_world(str(scene_world_path))
        if not sclerp.core.ok(st_load):
            raise RuntimeError(f"load_from_sdf_world failed: status={st_load}")

    wopt = sclerp.gazebo.WorldExportOptions()
    wopt.world_name = str(ns.world_name)

    robot = sclerp.gazebo.RobotModelFromUrdf()
    robot.urdf_path = str(urdf_path)
    robot.base_link = base_link
    robot.tip_link = tip_link
    robot.collapse_fixed_joints = collapse_fixed_joints
    robot.link_mesh_stl_directory = str(stl_dir)
    wopt.robot_from_urdf = robot

    playback = sclerp.gazebo.JointTrajectoryPlayback()
    playback.csv_path = str(csv_path)
    playback.loop = bool(ns.loop)
    playback.rate = float(ns.rate)
    wopt.joint_trajectory = playback

    if ns.plugin_dir and str(ns.plugin_dir):
        wopt.joint_trajectory_plugin_filename = str(Path(ns.plugin_dir).expanduser().resolve() / wopt.joint_trajectory_plugin_filename)

    world_path = out_dir / "world.sdf"
    st_world = reg.write_sdf_world(str(world_path), wopt)
    if not sclerp.core.ok(st_world):
        raise RuntimeError(f"write_sdf_world failed: status={st_world}")

    print("Wrote:")
    print(f"  dump:       {dump_path}")
    if ns.scene_world:
        print(f"  scene:      {Path(ns.scene_world).expanduser().resolve()}")
    print(f"  world:      {world_path}")
    print(f"  trajectory: {csv_path}")

    print_gazebo_hint(world_path)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as e:
        print(f"[visualize_failure_dump_gazebo.py] ERROR: {e}", file=sys.stderr)
        raise
