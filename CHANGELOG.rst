^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_models
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* MJCF follows the MuJoCo Menagerie standards: robot-only ``<robot>.xml`` with a
  ``home`` keyframe, ``scene.xml`` per robot, ramp and room scenes on shared
  terrains (``mujoco/scenes/scene_*.xml`` removed), per-robot README, CHANGELOG,
  LICENSE and preview image.
* MJCF actuators take ``ctrl`` in N·m and ``<joint>_trq`` reports joint torque;
  damping and friction no longer act on the floating base.
* Joint limits, armature and friction from the drive configs and datasheets;
  hb50 link masses and hb40 foot collision corrected.
* URDF cleanup: no zero-mass foot inertials or collision materials, invalid
  ``sim`` values rejected.
* Isaac Lab config updated to the Isaac Lab 3.0 API.
* Explicit ros2_control dependencies; new MuJoCo tests.
* RViz config: add the view tools so the camera can be moved.

0.1.0 (2026-09-30)
------------------
* Initial release.
* hb40, hb50 and hb50w models: xacro/URDF, MuJoCo MJCF with flat, ramp and room
  scenes, RBDL Lua models and visual meshes.
* RViz and Gazebo Sim launch files; ros2_control configs for mock and Gazebo.
* Isaac Lab articulation config for hb50.
* Tests keeping URDF, MJCF, RBDL and configs consistent.
