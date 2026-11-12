# Where the meshes and textures went

Two directories are **deliberately not in this repository**:

```
sim/src/disaster_models/models/            ~73 MB   victim, cone, barrel, first-aid meshes + textures
sim/src/unitree_go2_description/meshes/    ~28 MB   the Go2's own 3D meshes
```

They are Gazebo Fuel assets: not our work, not something we edit, and they do
not change. Committing them would put 100 MB of binaries into git history
permanently -- every clone would download them forever, even after a later
delete, and removing them would mean rewriting history and invalidating every
existing clone and pull request. git compresses binaries poorly, so each future
texture swap would add another full copy.

Everything else in `sim/src` is 3 MB of code, worlds, mission data and configs,
which is what we actually maintain.

## Getting them

`sim/build.sh` copies them in at build time from a checkout of
`SMC_SAR_Phase2`, which is where they came from:

```bash
cd sim && ./build.sh                          # expects ../../SMC_SAR_Phase2
GAZEBO_ASSETS=/path/to/SMC_SAR_Phase2 ./build.sh
```

The build stops with an explicit message if it cannot find them, rather than
producing an image whose robot and victims are invisible.
