add_library(node_base
  src/node_base/node_base.cpp
  src/node_base/parameter_handler.cpp
)

add_dependencies(node_base
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(node_base
  ${catkin_LIBRARIES}
)


