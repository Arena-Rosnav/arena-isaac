cmake_minimum_required(VERSION 3.8)
project(isaacsim_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/PrimPath.msg"
  "msg/Quat.msg"
  "msg/Euler.msg"
  "msg/Env.msg"
  "msg/Values.msg"
  "msg/ContactSensor.msg"
  )

set(srv_files
  "srv/ImportUsd.srv"
  "srv/UrdfToUsd.srv"
  "srv/ImportUrdf.srv"
  "srv/DeletePrim.srv"
  "srv/GetPrimAttributes.srv"
  "srv/MovePrim.srv"
  "srv/ImportYaml.srv"
  "srv/ScalePrim.srv"
  "srv/SpawnWall.srv"
  "srv/ImportObstacles.srv"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
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
