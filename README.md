# robot_models

Robot models of the [MAB Robotics](https://www.mabrobotics.pl/) Honey Badger quadrupeds, in one ROS 2 package:

- xacro / URDF for ROS 2, RViz and Gazebo Sim (with ros2_control)
- MuJoCo MJCF with flat, ramp and room scenes
- RBDL Lua models
- visual meshes (STL)
- an Isaac Lab articulation config (hb50)

Robot documentation: <https://mabrobotics.github.io/hb-docs/intro.html>

| Robot   | Type               | Actuated joints    | Mass     |
|---------|--------------------|--------------------|----------|
| [`hb40`](https://www.mabrobotics.pl/honey-badger)  | quadruped          | 12                 | 12.60 kg |
| [`hb50`](https://www.mabrobotics.pl/honey-badger-5)  | quadruped          | 12                 | 17.11 kg |
| `hb50w` | wheeled quadruped  | 16 (12 + 4 wheels) | 25.46 kg |

## Installation

Tested with ROS 2 Jazzy, Gazebo Harmonic and MuJoCo 3.4. Clone the repository
into the `src/` folder of a colcon workspace, then from the workspace root:

```
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select robot_models
source install/setup.bash
```

## Usage

### RViz

```
ros2 launch robot_models view.launch.py robot:=hb50
```

Shows the robot with joint sliders. The fixed frame is `body`.

### Gazebo Sim

```
ros2 launch robot_models gazebo.launch.py robot:=hb50
```

Spawns the robot in an empty world with ros2_control (`gz_ros2_control`) and
starts:

- `joint_state_broadcaster`
- `forward_position_controller`: position commands for the 12 leg joints on
  `/forward_position_controller/commands` (`std_msgs/Float64MultiArray`, joint
  order as in `config/<robot>/ros2_control.yaml`)
- `wheel_velocity_controller` (hb50w only): velocity commands for the wheels
- a bridge for `/clock` and the body IMU on `/imu` (`sensor_msgs/Imu`)

### MuJoCo

```
pip install mujoco
python3 -m mujoco.viewer --mjcf mujoco/hb50/scene.xml
```

The MJCF follows the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
standards. `mujoco/<robot>/<robot>.xml` is the robot alone. `scene.xml` adds a flat floor;
`scene_ramp.xml` and `scene_room.xml` combine the robot with the terrains in
`mujoco/scenes/`, which all robots share. The `home` keyframe is the standing
pose.

Actuators are torque `motor`s named after their joint. Sensors: `<joint>_pos`, `<joint>_vel` and
`<joint>_trq` for every joint, plus `torso-orientation`,
`torso-angular-velocity`, `torso-linear-acceleration` and `torso-magnetometer`
on the `imu` site.

### RBDL

`rbdl/<robot>/<robot>.lua` loads with RBDL's LuaModel addon. The root body
`base` has a 6-DoF free-flyer joint; every other body is named after the joint
that moves it (`fr_j0`, `fr_j1`, ...), and `<leg>_foot` is a fixed frame at
the foot (at the wheel centre for hb50w).

### Isaac Sim / Isaac Lab

Convert the URDF to USD with Isaac Sim's
[URDF Importer](https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/ext_isaacsim_asset_importer_urdf.html).
This package provides:

- `urdf/hb50/hb50.urdf`: the URDF to convert
- `usd/hb50/isaaclab/hb50_cfg.py`: an Isaac Lab articulation config
  (`HB50_CFG`) that loads the converted model from `usd/hb50/hb50.usd` and sets
  the joint armature, which URDF cannot express

## Model conventions

- SI units, angles in radians; x forward, y left, z up.
- Legs: `fr`, `fl`, `rl`, `rr`. Joints: `<leg>_j0` hip abduction, `<leg>_j1`
  hip flexion, `<leg>_j2` knee, `<leg>_j3` wheel (hb50w).
- Links: root `body` (free-floating, no `base_link`), `<leg>_l0`, `<leg>_l1`,
  `<leg>_l2`, and `<leg>_foot`, or `<leg>_w` for the hb50w wheels.
- URDF, MJCF and the ros2_control configs use the same names.

### xacro arguments

`urdf/<robot>/<robot>.urdf.xacro` takes a `sim` argument:

- `none` (default): plain URDF, identical to the committed `<robot>.urdf`
- `mock`: adds a ros2_control block with `mock_components/GenericSystem`
- `gazebo`: adds a ros2_control block with `gz_ros2_control`, the Gazebo
  plugins and an IMU sensor

Any other value stops xacro with an error.

## Repository layout

`<robot>` is `hb40`, `hb50` or `hb50w`.

```
robot_models
│
└───urdf
│   └───common          shared xacro macros: materials, ros2_control, Gazebo
│   └───<robot>         xacro robot description and the generated plain URDF
│
└───meshes
│   └───<robot>/visual  visual STL meshes
│
└───mujoco
│   └───scenes          ramp and room terrains shared by all robots
│   └───planes          height maps for the terrains
│   └───<robot>         MJCF robot model and its scenes
│
└───rbdl
│   └───<robot>         RBDL Lua model
│
└───config
│   └───<robot>         ros2_control and joint limit configs
│
└───usd
│   └───hb50            Isaac Lab articulation config
│
└───launch              RViz and Gazebo Sim launch files
└───rviz                RViz config
└───scripts             URDF generation script
└───test                consistency tests
```

## Development

The MJCF is the numeric reference. When you change a model, update the MJCF,
the xacro, the Lua model and `config/<robot>/joint_limits.yaml` together, then
regenerate the plain URDFs and run the tests:

```
scripts/generate_urdf.sh
python3 -m pytest test/
```

Commit the regenerated `.urdf` files. Both the script and the tests need `xacro`
on `PATH`; the tests do not need the package to be built or sourced. They check:

- URDF vs MJCF: inertials, joint frames, axes, limits and effort, visual and
  collision geometry, the Gazebo IMU pose against the MJCF `imu` site, and the
  `<joint>`, `<joint>_pos`, `<joint>_vel` and `<joint>_trq` names
- URDF vs RBDL: inertials, joint frames and axes, total mass (skipped without
  `lua5.1`)
- `config/<robot>/*.yaml` joint names and limits vs URDF
- that every xacro variant expands and the committed URDFs are up to date
- that every MuJoCo scene runs without warnings under random controls, and the
  `home` keyframe stands on the floor (skipped without the `mujoco` Python
  package)
- that each `mujoco/<robot>/` folder has its README, CHANGELOG, LICENSE and
  preview image, and the mesh paths its README describes

`colcon test --packages-select robot_models` runs the same tests plus the ament
linters.

## Known limitations

- The Isaac Lab config exists for hb50 only, and its default pose is a
  placeholder.
- `config/<robot>/joint_limits.yaml` lists the URDF joint limits for reference;
  the launch files do not load it.
- robot_state_publisher warns that the root link `body` has an inertia (a KDL
  limitation); the warning is harmless.

## License

Apache-2.0, see [LICENSE](LICENSE).
