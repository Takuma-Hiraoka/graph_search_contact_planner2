cmake_minimum_required(VERSION 3.16)
project(graph_search_contact_planner_sample)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
find_package(ament_cmake REQUIRED)
set(dependencies
  ament_index_cpp
  graph_search_contact_planner
  ik_constraint
  ik_constraint_coal
  mujoco_viewer
  pinocchio)
foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()
add_library(sample_common src/g1_common.cpp src/world_common.cpp)
target_include_directories(sample_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(sample_common ${dependencies})
target_link_libraries(sample_common pinocchio::pinocchio)
add_executable(sample0 src/sample0.cpp)
ament_target_dependencies(sample0 ${dependencies})
target_link_libraries(sample0 sample_common pinocchio::pinocchio)
install(TARGETS sample_common EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS sample0 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h")
install(DIRECTORY unitree_ros DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()
