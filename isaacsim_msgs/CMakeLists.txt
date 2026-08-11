cmake_minimum_required(VERSION 3.8)
project(isaacsim_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Ceiling.msg
  msg/ContactSensor.msg
  msg/Door.msg
  msg/Elevator.msg
  msg/Floor.msg
  msg/Material.msg
  msg/PhysicsParams.msg
  msg/Prim.msg
  msg/Scale.msg
  msg/Wall.msg
  srv/DeletePrims.srv
  srv/EditPrims.srv
  srv/GetPrims.srv
  srv/ResetWorld.srv
  srv/SpawnCeilings.srv
  srv/SpawnFloors.srv
  srv/SpawnPrims.srv
  srv/SpawnUrdf.srv
  srv/SpawnUsd.srv
  srv/SpawnWalls.srv
  srv/StepSimulation.srv
  DEPENDENCIES std_msgs geometry_msgs
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
