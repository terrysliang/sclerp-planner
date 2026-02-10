# sclerp-planner
ScLERP-based motion planning stack with collision avoidance.

This repo is split into three main modules:
- `core/`: Dual quaternions, POE kinematics, RMRC, and a local SE(3) ScLERP tracking planner (`planMotionSclerp`).
- `urdf/`: Loads a serial chain from URDF into the minimal `core::ManipulatorModel`.
- `collision/`: Wraps the core planner with FCL-based closest-contact queries and an LCP-based
  joint correction step (`planMotionSclerpWithCollision`).

Conventions used throughout:
- `Transform` is `Eigen::Isometry3d` (SE(3)), composed by left-multiplication.
- Twists/screw axes use ordering `[v; w]` (linear; angular) in the space/world frame.
- Units: meters and radians.
- `rotationDistance` is quaternion chordal distance (not angle in radians).

References:
- Anirban Sinha, Anik Sarker, Nilanjan Chakraborty, "Task Space Planning with Complementarity Constraint-based Obstacle Avoidance", arXiv:2104.07849v1 [cs.RO], 16 Apr 2021. https://arxiv.org/abs/2104.07849
