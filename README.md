# robot_models

Robot models of the MAB Robotics Honey Badger quadrupeds, in one ROS 2 package:

- xacro / URDF for ROS 2, RViz and Gazebo Sim (with ros2_control)
- MuJoCo MJCF with flat, ramp and room scenes
- RBDL Lua models
- visual meshes (STL)
- an Isaac Lab articulation config (hb50)

Robot documentation: <https://mabrobotics.github.io/hb-docs/intro.html>

| Robot   | Type               | Actuated joints    | Mass     |
|---------|--------------------|--------------------|----------|
| `hb40`  | quadruped          | 12                 | 12.60 kg |
| `hb50`  | quadruped          | 12                 | 14.75 kg |
| `hb50w` | wheeled quadruped  | 16 (12 + 4 wheels) | 25.46 kg |

## Installation

Tested with ROS 2 Jazzy, Gazebo Harmonic and MuJoCo 3.4. Clone the repository
into the `src/` folder of a colcon workspace, then from the workspace root:

```
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select robot_models
source install/setup.bash
```

The plain URDFs (`urdf/<robot>/<robot>.urdf`) and the MJCF files can also be
used straight from the repository, without building.

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
python3 -m mujoco.viewer --mjcf mujoco/hb50/hb50.xml
```

Each robot file includes `mujoco/scenes/scene_flat.xml`; change that
`<include>` to `scene_ramp.xml` or `scene_room.xml` for the other scenes.

Actuators are torque `motor`s named after their joint (`ctrl` = joint torque /
`gear`). Sensors: `<joint>_pos`, `<joint>_vel` and `<joint>_trq` for every
joint, plus `torso-orientation`, `torso-angular-velocity`,
`torso-linear-acceleration` and `torso-magnetometer` on the `imu` site.

### RBDL

`rbdl/<robot>/<robot>.lua` loads with RBDL's LuaModel addon. The root body
`base` has a 6-DoF free-flyer joint; every other body is named after the joint
that moves it (`fr_j0`, `fr_j1`, ...), and `<leg>_foot` is a fixed frame at
the foot (at the wheel centre for hb50w).

### Isaac Lab

Convert `urdf/hb50/hb50.urdf` to `usd/hb50/hb50.usd` with Isaac Lab's URDF
converter (`scripts/tools/convert_urdf.py` in the Isaac Lab repository), then
use `HB50_CFG` from `usd/hb50/isaaclab/hb50_cfg.py`. The joint armature, which
URDF cannot express, is set there.

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

## Repository layout

```
urdf/<robot>/      <robot>.urdf.xacro, leg macro, generated <robot>.urdf
urdf/common/       materials, ros2_control and Gazebo macros
meshes/<robot>/    visual STLs
mujoco/<robot>/    MJCF
mujoco/scenes/     scene_flat, scene_ramp, scene_room
mujoco/planes/     height maps for the ramp and room scenes
rbdl/<robot>/      RBDL Lua model
config/<robot>/    ros2_control.yaml, joint_limits.yaml
usd/hb50/          Isaac Lab config
launch/            view.launch.py (RViz), gazebo.launch.py
rviz/              view.rviz
scripts/           generate_urdf.sh
test/              consistency tests
```

## Development

The MJCF is the numeric reference. When you change a model, update the MJCF,
the xacro and the Lua model together, then regenerate the plain URDFs and run
the tests:

```
scripts/generate_urdf.sh
python3 -m pytest test/
```

`generate_urdf.sh` needs `xacro`; commit the regenerated `.urdf` files. The
tests do not need the package to be built or sourced. They check:

- URDF vs MJCF: inertials, joint frames, axes, limits and effort, visual and
  collision geometry, the Gazebo IMU pose against the MJCF `imu` site, and the
  actuator and sensor names listed above
- URDF vs RBDL: inertials, joint frames and axes, total mass (skipped without
  `lua5.1`)
- `config/<robot>/*.yaml` joint names and limits vs URDF
- that every xacro variant expands and the committed URDFs are up to date

`colcon test --packages-select robot_models` runs the same tests plus the ament
linters.

## Known limitations

- The hb50w RBDL model has no wheel bodies, so it is 0.94 kg lighter than the
  URDF and MJCF (24.52 kg vs 25.46 kg).
- Collision geometry uses primitives; there are no collision meshes.
- The Isaac Lab config exists for hb50 only, and its default pose and PD gains
  are placeholders.
- `config/<robot>/joint_limits.yaml` lists the URDF joint limits for reference;
  the launch files do not load it.
- robot_state_publisher warns that the root link `body` has an inertia (a KDL
  limitation); the warning is harmless.

## License

Apache-2.0, see [LICENSE](LICENSE).
