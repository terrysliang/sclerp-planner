from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]


DIANA7_PRESET_Q_INIT = np.array(
    [
        1.886428995278953,
        0.12622070334762633,
        0.036244670128887949,
        2.4665118153771788,
        -0.3020034302671557,
        -0.5708880960015188,
        2.208232253505976,
    ],
    dtype=np.float64,
)


@dataclass
class CommonArgs:
    urdf_path: Path
    stl_dir: Path
    base_link: str
    tip_link: str

    out_dir: Path
    world_name: str

    q_init: Optional[np.ndarray]

    dx: float
    dy: float
    dz: float

    sample_dt: float
    vmax: float
    amax: float

    loop: bool
    rate: float

    plugin_dir: Optional[Path]
    rmrc_nullspace: bool


def _parse_csv_floats(s: str) -> np.ndarray:
    xs = [t.strip() for t in s.split(",")]
    if any(not t for t in xs):
        raise ValueError("empty element")
    return np.array([float(t) for t in xs], dtype=np.float64)


def _bundled_diana_or_throw() -> CommonArgs:
    assets_dir = REPO_ROOT / "examples" / "assets"
    urdf_path = assets_dir / "diana7" / "urdf" / "diana_robot.urdf"
    stl_dir = assets_dir / "diana7" / "meshes"
    if not urdf_path.is_file():
        raise RuntimeError(f"Bundled demo URDF not found: {urdf_path}")
    if not stl_dir.is_dir():
        raise RuntimeError(f"Bundled demo STL directory not found: {stl_dir}")
    return CommonArgs(
        urdf_path=urdf_path.resolve(),
        stl_dir=stl_dir.resolve(),
        base_link="base_link",
        tip_link="link7",
        out_dir=Path("sclerp_py_out").resolve(),
        world_name="sclerp_world",
        q_init=DIANA7_PRESET_Q_INIT.copy(),
        dx=-0.15,
        dy=-0.184,
        dz=0.059,
        sample_dt=0.01,
        vmax=1.0,
        amax=2.0,
        loop=False,
        rate=1.0,
        plugin_dir=None,
        rmrc_nullspace=False,
    )


def add_common_cli(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--urdf", type=str, default=None)
    parser.add_argument("--stl-dir", type=str, default=None)
    parser.add_argument("--base-link", type=str, default=None)
    parser.add_argument("--tip-link", type=str, default=None)
    parser.add_argument("--q-init", type=str, default=None, help="CSV list, size must match DOF")

    parser.add_argument("--out-dir", type=str, default="sclerp_py_out")
    parser.add_argument("--world-name", type=str, default="sclerp_world")
    parser.add_argument("--plugin-dir", type=str, default=None, help="Directory containing libsclerp_joint_trajectory_player.so")

    parser.add_argument("--dx", type=float, default=-0.15)
    parser.add_argument("--dy", type=float, default=-0.184)
    parser.add_argument("--dz", type=float, default=0.059)

    parser.add_argument("--sample-dt", type=float, default=0.01)
    parser.add_argument("--vmax", type=float, default=1.0)
    parser.add_argument("--amax", type=float, default=2.0)

    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--loop", action="store_true")

    parser.add_argument("--rmrc-nullspace", action="store_true", help="Enable RMRC nullspace joint-limit+posture terms (dof>6 only)")


def parse_common_args_or_throw(ns: argparse.Namespace) -> CommonArgs:
    has_any_robot_arg = any([ns.urdf, ns.stl_dir, ns.base_link, ns.tip_link])
    if not has_any_robot_arg:
        out = _bundled_diana_or_throw()
    else:
        missing = [k for k in ["urdf", "stl_dir", "base_link", "tip_link"] if getattr(ns, k) in (None, "")]
        if missing:
            raise RuntimeError(
                "When providing robot arguments, all of --urdf, --stl-dir, --base-link, --tip-link must be set. "
                f"Missing: {', '.join('--' + m.replace('_', '-') for m in missing)}"
            )

        out = CommonArgs(
            urdf_path=Path(ns.urdf).expanduser().resolve(),
            stl_dir=Path(ns.stl_dir).expanduser().resolve(),
            base_link=str(ns.base_link),
            tip_link=str(ns.tip_link),
            out_dir=Path(ns.out_dir).expanduser().resolve(),
            world_name=str(ns.world_name),
            q_init=None,
            dx=float(ns.dx),
            dy=float(ns.dy),
            dz=float(ns.dz),
            sample_dt=float(ns.sample_dt),
            vmax=float(ns.vmax),
            amax=float(ns.amax),
            loop=bool(ns.loop),
            rate=float(ns.rate),
            plugin_dir=Path(ns.plugin_dir).expanduser().resolve() if ns.plugin_dir else None,
            rmrc_nullspace=bool(ns.rmrc_nullspace),
        )

    if ns.q_init:
        out.q_init = _parse_csv_floats(ns.q_init)

    out.out_dir = Path(ns.out_dir).expanduser().resolve()
    out.world_name = str(ns.world_name)
    out.dx = float(ns.dx)
    out.dy = float(ns.dy)
    out.dz = float(ns.dz)
    out.sample_dt = float(ns.sample_dt)
    out.vmax = float(ns.vmax)
    out.amax = float(ns.amax)
    out.rate = float(ns.rate)
    out.loop = bool(ns.loop)
    out.rmrc_nullspace = bool(ns.rmrc_nullspace)
    out.plugin_dir = Path(ns.plugin_dir).expanduser().resolve() if ns.plugin_dir else None

    if not (out.sample_dt > 0.0) or not math.isfinite(out.sample_dt):
        raise RuntimeError("--sample-dt must be > 0")
    if not (out.vmax > 0.0) or not math.isfinite(out.vmax):
        raise RuntimeError("--vmax must be > 0")
    if not (out.amax > 0.0) or not math.isfinite(out.amax):
        raise RuntimeError("--amax must be > 0")
    if not (out.rate > 0.0) or not math.isfinite(out.rate):
        raise RuntimeError("--rate must be > 0")

    return out


def ensure_output_dir(out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)


def maybe_midpoint_q_init(solver, q_init: np.ndarray) -> np.ndarray:
    # Mirror the C++ heuristic used by examples: if q=0 is outside/near a limit, use midpoint.
    dof = solver.model().dof()
    if q_init.shape != (dof,):
        raise RuntimeError(f"q_init must have shape ({dof},), got {q_init.shape}")

    q = q_init.copy()
    near_limit_eps = 1e-3
    for i in range(dof):
        lim = solver.model().joint(i).limit
        if not lim.enabled:
            continue
        if not (math.isfinite(lim.lower) and math.isfinite(lim.upper)):
            continue
        if not (lim.lower <= lim.upper):
            continue

        q0 = 0.0
        outside = (q0 < lim.lower) or (q0 > lim.upper)
        near_bound = (abs(q0 - lim.lower) < near_limit_eps) or (abs(lim.upper - q0) < near_limit_eps)
        if outside or near_bound:
            q[i] = 0.5 * (lim.lower + lim.upper)
    return q


def rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    return (Rz @ Ry @ Rx).astype(np.float64)


def print_gazebo_hint(world_path: Path) -> None:
    print("\nRun Gazebo:")
    guess_plugin_dir = REPO_ROOT / "build" / "gazebo"
    if guess_plugin_dir.is_dir():
        print(f'  export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="{guess_plugin_dir}:${{IGN_GAZEBO_SYSTEM_PLUGIN_PATH}}"')
    else:
        print('  export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="<sclerp_build>/gazebo:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"')
    print(f'  ign gazebo "{world_path}"')


