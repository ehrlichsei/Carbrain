##Declare Node
set(SOURCES
  node_helper/pavlov_node_helper.cpp
  node_helper/driving_corridor_checker.cpp
  node_helper/turn_checker.cpp
  parking_fsm/data_parking.cpp
  fsm_utils/carcontroller.cpp
  fsm_utils/diagnostics_interface.cpp
  fsm_utils/timeout.cpp
  obstacle_fsm/shared_data.cpp)

add_executable(obstacle_pavlov    
  ${SOURCES}
  obstacle_pavlov_node.cpp
)
add_executable(free_drive_pavlov  
  ${SOURCES}
  free_drive_pavlov_node.cpp
)

##Declare Nodelet
add_library(obstacle_pavlov_nodelet
  ${SOURCES}
  obstacle_pavlov_node.cpp
)
add_library(free_drive_pavlov_nodelet  
  ${SOURCES}
  free_drive_pavlov_node.cpp
)

include_directories(fsm_utils)
include_directories(obstacle_fsm)
include_directories(parking_fsm)
include_directories(node_helper)

## set compiler definitions for nodelet
set_target_properties(free_drive_pavlov_nodelet obstacle_pavlov_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(obstacle_pavlov
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(free_drive_pavlov
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(free_drive_pavlov_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(obstacle_pavlov_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(obstacle_pavlov
  ${catkin_LIBRARIES}
)
target_link_libraries(free_drive_pavlov
  ${catkin_LIBRARIES}
)
target_link_libraries(free_drive_pavlov_nodelet
  ${catkin_LIBRARIES}
)
target_link_libraries(obstacle_pavlov_nodelet
  ${catkin_LIBRARIES}
)
