# CMake generated Testfile for 
# Source directory: /home/kien/Documents/Arena4-IsaacSim/src/isaacsim_msgs
# Build directory: /home/kien/Documents/Arena4-IsaacSim/build/isaacsim_msgs
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(lint_cmake "/home/kien/Documents/Arena4-IsaacSim/venv/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/kien/Documents/Arena4-IsaacSim/build/isaacsim_msgs/test_results/isaacsim_msgs/lint_cmake.xunit.xml" "--package-name" "isaacsim_msgs" "--output-file" "/home/kien/Documents/Arena4-IsaacSim/build/isaacsim_msgs/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/humble/bin/ament_lint_cmake" "--xunit-file" "/home/kien/Documents/Arena4-IsaacSim/build/isaacsim_msgs/test_results/isaacsim_msgs/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/kien/Documents/Arena4-IsaacSim/src/isaacsim_msgs" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/kien/Documents/Arena4-IsaacSim/src/isaacsim_msgs/CMakeLists.txt;46;ament_package;/home/kien/Documents/Arena4-IsaacSim/src/isaacsim_msgs/CMakeLists.txt;0;")
add_test(xmllint "/home/kien/Documents/Arena4-IsaacSim/venv/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/kien/Documents/Arena4-IsaacSim/build/isaacsim_msgs/test_results/isaacsim_msgs/xmllint.xunit.xml" "--package-name" "isaacsim_msgs" "--output-file" "/home/kien/Documents/Arena4-IsaacSim/build/isaacsim_msgs/ament_xmllint/xmllint.txt" "--command" "/opt/ros/humble/bin/ament_xmllint" "--xunit-file" "/home/kien/Documents/Arena4-IsaacSim/build/isaacsim_msgs/test_results/isaacsim_msgs/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/kien/Documents/Arena4-IsaacSim/src/isaacsim_msgs" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/kien/Documents/Arena4-IsaacSim/src/isaacsim_msgs/CMakeLists.txt;46;ament_package;/home/kien/Documents/Arena4-IsaacSim/src/isaacsim_msgs/CMakeLists.txt;0;")
subdirs("isaacsim_msgs__py")
