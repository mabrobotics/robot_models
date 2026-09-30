cmake_minimum_required(VERSION 3.16)
project(robot_models)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    urdf
    meshes
    mujoco
    usd
    config
    launch
    rviz
    rbdl
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_xacro_validity test/test_xacro_validity.py)
  ament_add_pytest_test(test_urdf_matches_mjcf test/test_urdf_matches_mjcf.py)
  ament_add_pytest_test(test_urdf_matches_rbdl test/test_urdf_matches_rbdl.py)
endif()

ament_package()
