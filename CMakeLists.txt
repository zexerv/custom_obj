cmake_minimum_required(VERSION 3.0.2)
project(moveit_custom_objective_plugin)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  moveit_core
  moveit_ros_planning     # For planning_interface, planning_scene etc.
  moveit_planners_ompl  # For ompl_interface classes
  tf2_eigen             # Added dependency
  eigen_conversions     # Added dependency
)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS system) # For pluginlib

# --- DEBUG MESSAGES (Keep these) ---
message(STATUS "DEBUG (${PROJECT_NAME}): catkin_INCLUDE_DIRS = ${catkin_INCLUDE_DIRS}")
message(STATUS "DEBUG (${PROJECT_NAME}): moveit_ros_planning_INCLUDE_DIRS = ${moveit_ros_planning_INCLUDE_DIRS}")
message(STATUS "DEBUG (${PROJECT_NAME}): moveit_planners_ompl_INCLUDE_DIRS = ${moveit_planners_ompl_INCLUDE_DIRS}")
message(STATUS "DEBUG (${PROJECT_NAME}): ompl_INCLUDE_DIRS = ${ompl_INCLUDE_DIRS}")
message(STATUS "DEBUG (${PROJECT_NAME}): Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}")
# --- END DEBUG MESSAGES ---

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core # Library name changed
  CATKIN_DEPENDS roscpp pluginlib moveit_core moveit_ros_planning moveit_planners_ompl tf2_eigen eigen_conversions
  DEPENDS Boost ompl
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${ompl_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

# Build the shared library
add_library(${PROJECT_NAME}_core src/custom_ompl_planner_manager.cpp)
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${ompl_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES OUTPUT_NAME "moveit_custom_objective_plugin_core")
install(FILES custom_ompl_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
file(COPY custom_ompl_plugin_description.xml
     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})
# --- Installation rules ---
install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# *** MODIFIED INSTALL SECTION FOR XML ***
# Try installing the directory containing the XML file using a pattern match
message(STATUS "Attempting to install XML via DIRECTORY command...")
install(DIRECTORY . # Use "." for the current source directory (where CMakeLists.txt is)
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "custom_ompl_plugin_description.xml"
)
# *** END MODIFIED INSTALL SECTION ***