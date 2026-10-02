# MAB Robotics HB50 Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 2.3.7 or later.

## Changelog

See [CHANGELOG.md](./CHANGELOG.md) for a full history of changes.

## Overview

This package contains a robot description (MJCF) of the [HB50](https://www.mabrobotics.pl/honey-badger-5) quadruped
developed by [MAB Robotics](https://www.mabrobotics.pl/). It derives from the MJCF in
[robot_models](https://github.com/mabrobotics/robot_models).

<p float="left">
  <img src="hb50.png" width="400">
</p>

## Derivation steps

1. Copied `mujoco/hb50/` from robot_models.
2. Removed `scene_ramp.xml` and `scene_room.xml`, which use terrains shared between the robots in
   robot_models.
3. Copied the meshes from `meshes/hb50/visual/` into `assets/`.
4. Set `meshdir="assets"` in `<compiler>` and removed the `../../meshes/hb50/visual/` prefix from the
   mesh files.

## License

This model is released under an [Apache-2.0 License](LICENSE).
