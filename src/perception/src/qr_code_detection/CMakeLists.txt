if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  set(qr_code_detection_no_debug "TRUE")
endif ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")

find_package(PkgConfig REQUIRED)
pkg_check_modules(zbar REQUIRED zbar)

add_library(qr_code_detection_lib
  src/qr_code_detection/qr_code_detection.cpp
)

##Declare Node
add_executable(qr_code_detection
  src/qr_code_detection/qr_code_detection_node.cpp
)

##Declare Nodelet
add_library(qr_code_detection_nodelet
  src/qr_code_detection/qr_code_detection_node.cpp
)
## set compiler definitions for nodelet
set_target_properties(qr_code_detection_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

if(${PROJECT_NAME}_gencfg)
  add_dependencies(qr_code_detection ${PROJECT_NAME}_gencfg)
  add_dependencies(qr_code_detection_nodelet ${PROJECT_NAME}_gencfg)
endif(${PROJECT_NAME}_gencfg)

add_dependencies(qr_code_detection ${catkin_EXPORTED_TARGETS})
add_dependencies(qr_code_detection_nodelet ${catkin_EXPORTED_TARGETS})


target_link_libraries(qr_code_detection_lib
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${zbar_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_include_directories(qr_code_detection_lib PUBLIC ${zbar_INCLUDE_DIRS})

target_link_libraries(qr_code_detection
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  qr_code_detection_lib
)
target_link_libraries(qr_code_detection_nodelet
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  qr_code_detection_lib
)

## set compiler definitions for nodelet
set_target_properties(qr_code_detection_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

if (NOT qr_code_detection_no_debug)
  add_library(qr_code_detection_debug_lib STATIC
    src/qr_code_detection/debug/qr_code_detection_node_debug.cpp
  )

  add_dependencies(qr_code_detection_debug_lib ${catkin_EXPORTED_TARGETS})
  target_link_libraries(qr_code_detection_debug_lib ${OpenCV_LIBRARIES})

  target_link_libraries(qr_code_detection
    qr_code_detection_debug_lib
  )

else(NOT qr_code_detection_no_debug)
  set_target_properties(qr_code_detection PROPERTIES COMPILE_DEFINITIONS "NO_FANCY_DEBUG")
endif(NOT qr_code_detection_no_debug)

