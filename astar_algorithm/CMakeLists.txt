cmake_minimum_required(VERSION 3.5)
project(astar_algorithm)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# install(DIRECTORY
#   # launch
#   src
#   scripts
#   # param
#   # remote_control
#   DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY
    scripts 
    src
    # launch
  DESTINATION
    share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python executables
# install(PROGRAMS
#   # src/mte544_navigation_server.py
#   # src/mte544_navigation_client.py
#   # src/a_star_skeleton1.py 
#   # src/astar_test1.py
#   # src/astar_test2.py
#   # src/astar_test3.py
#   # src/astar_test4.py
#   # src/astar_test5.py
  # src/astar_test6.py
#   scripts/pure_pursuit_yolo.py
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  PROGRAMS
  # scripts/sign_detect.py
  # scripts/sign_distance.py
  src/astar_origi.py
  src/pure_pursuit_yolo.py
  src/pure_pursuit_yolo02.py
  src/pure_pursuit_control04.py
  # launch/launch_yolov8.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
