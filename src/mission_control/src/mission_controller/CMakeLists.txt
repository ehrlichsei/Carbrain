##Declare Node
add_executable(mission_controller
  mission_controller_node.cpp
)

##Declare Nodelet
add_library(mission_controller_nodelet
  mission_controller_node.cpp
)

set_target_properties(mission_controller PROPERTIES COMPILE_FLAGS "${OpenMP_CXX_FLAGS}")
set_target_properties(mission_controller PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

## set compiler definitions for nodelet
set_target_properties(mission_controller_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")
set_target_properties(mission_controller_nodelet PROPERTIES COMPILE_FLAGS "${OpenMP_CXX_FLAGS}")
set_target_properties(mission_controller_nodelet PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

add_dependencies(mission_controller
#  ${PROJECT_NAME}_gencfg
  mission_control_gencpp
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(mission_controller_nodelet
#  ${PROJECT_NAME}_gencfg
  mission_control_gencpp
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mission_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(mission_controller_nodelet
  ${catkin_LIBRARIES}
)

if(CMAKE_CXX_CLANG_TIDY)
 set_target_properties(mission_controller PROPERTIES CXX_CLANG_TIDY "")
 set_target_properties(mission_controller_nodelet PROPERTIES CXX_CLANG_TIDY "")
endif()
