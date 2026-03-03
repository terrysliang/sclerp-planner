# Diana7 example assets

This folder contains a small set of robot assets bundled for running the `sclerp_example_*` executables out-of-the-box:

- `urdf/diana_robot.urdf`
- `meshes/*.STL` (simplified link meshes)

## Mesh naming

The URDF link names are `base_link`, `link1`, …, `link7`.

The simplified mesh set we bundle originates as `link0.STL` … `link7.STL`, so we apply the following mapping:

- `link0.STL` → `base_link.STL`
- `link1.STL` … `link7.STL` → unchanged

## License

These assets are provided under the same license as the repository: Apache-2.0 (see `LICENSE` at repo root).

