add_library(localization_objs OBJECT
  src/localization/localization.cpp
)

##Declare Node
add_executable(localization
  $<TARGET_OBJECTS:localization_objs>
  src/localization/localization_node.cpp
)

##Declare Nodelet
add_library(localization_nodelet
  $<TARGET_OBJECTS:localization_objs>
  src/localization/localization_node.cpp
)
## set compiler definitions for nodelet
set_target_properties(localization_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(localization_objs ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(localization ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(localization_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(localization
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)
target_link_libraries(localization_nodelet
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)
