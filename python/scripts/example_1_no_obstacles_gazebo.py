#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

import sclerp

from example_common import add_common_cli, ensure_output_dir, maybe_midpoint_q_init, parse_common_args_or_throw, print_gazebo_hint


def main() -> int:
    p = argparse.ArgumentParser(description="Example 1 (Python): plan without obstacles, TOTG, export Gazebo world + CSV.")
    add_common_cli(p)
    ns = p.parse_args()

    args = parse_common_args_or_throw(ns)
    ensure_output_dir(args.out_dir)

    # 1) Load URDF -> ManipulatorModel.
    uopt = sclerp.urdf.LoadOptions()
    uopt.base_link = args.base_link
    uopt.tip_link = args.tip_link
    uopt.collapse_fixed_joints = True
    uopt.strict = True

    load = sclerp.urdf.load_manipulator_model_from_file(str(args.urdf_path), uopt)
    if not sclerp.core.ok(load.status):
        raise RuntimeError(f"load_manipulator_model_from_file failed: {load.message}")

    # 2) Build kinematics solver.
    solver = sclerp.core.KinematicsSolver(load.model)
    dof = solver.model().dof()

    # 3) Choose q_init.
    if args.q_init is None:
        q_init = maybe_midpoint_q_init(solver, np.zeros((dof,), dtype=np.float64))
    else:
        q_init = np.asarray(args.q_init, dtype=np.float64).reshape(-1)
        if q_init.shape != (dof,):
            raise RuntimeError(f"--q-init must have exactly {dof} values; got {q_init.size}")

    # 4) Start pose -> translated goal pose.
    st_fk, g_i = solver.forward_kinematics(q_init)
    if not sclerp.core.ok(st_fk):
        raise RuntimeError("forward_kinematics failed")

    g_f = np.array(g_i, copy=True)
    g_f[:3, 3] += np.array([args.dx, args.dy, args.dz], dtype=np.float64)

    opt = sclerp.core.MotionPlanOptions()
    if args.rmrc_nullspace:
        opt.rmrc.nullspace.enabled = True
        opt.rmrc.nullspace.joint_limits.enabled = True
        opt.rmrc.nullspace.posture.enabled = True

    result = sclerp.core.plan_motion_sclerp(solver, q_init, g_f, opt)
    if not sclerp.core.ok(result.status):
        raise RuntimeError(f"plan_motion_sclerp failed: status={result.status}")

    # 5) Time-parameterize (TOTG).
    lim = sclerp.trajectory.Limits()
    lim.v_max = np.full((dof,), args.vmax, dtype=np.float64)
    lim.a_max = np.full((dof,), args.amax, dtype=np.float64)

    cfg = sclerp.trajectory.GridTotg.Config()
    cfg.unwrap_angles = True

    totg = sclerp.trajectory.GridTotg(lim, cfg)
    unwrap_mask = sclerp.trajectory.continuous_revolute_mask(solver.model())

    st_totg, traj = totg.plan(result.path, args.sample_dt, unwrap_mask)
    if not sclerp.core.ok(st_totg):
        raise RuntimeError(f"GridTotg.plan failed: status={st_totg}")

    csv_path = args.out_dir / "trajectory.csv"
    st_csv = sclerp.trajectory.write_trajectory_csv(traj, str(csv_path), sclerp.trajectory.CsvMode.PositionOnly)
    if not sclerp.core.ok(st_csv):
        raise RuntimeError(f"write_trajectory_csv failed: status={st_csv}")

    # 6) Export playback world.
    reg = sclerp.gazebo.WorldRegistry()
    wopt = sclerp.gazebo.WorldExportOptions()
    wopt.world_name = args.world_name

    robot = sclerp.gazebo.RobotModelFromUrdf()
    robot.urdf_path = str(args.urdf_path)
    robot.base_link = args.base_link
    robot.tip_link = args.tip_link
    robot.collapse_fixed_joints = True
    robot.link_mesh_stl_directory = str(args.stl_dir)
    wopt.robot_from_urdf = robot

    playback = sclerp.gazebo.JointTrajectoryPlayback()
    playback.csv_path = str(csv_path)
    playback.loop = args.loop
    playback.rate = args.rate
    wopt.joint_trajectory = playback

    if args.plugin_dir and str(args.plugin_dir):
        wopt.joint_trajectory_plugin_filename = str(Path(args.plugin_dir) / wopt.joint_trajectory_plugin_filename)

    world_path = args.out_dir / "world.sdf"
    st_world = reg.write_sdf_world(str(world_path), wopt)
    if not sclerp.core.ok(st_world):
        raise RuntimeError(f"write_sdf_world failed: status={st_world}")

    print("Wrote:")
    print(f"  world:      {world_path}")
    print(f"  trajectory: {csv_path}")

    print_gazebo_hint(world_path)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as e:
        print(f"[example_1_no_obstacles_gazebo.py] ERROR: {e}", file=sys.stderr)
        raise

