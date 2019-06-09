## This function creates a traget, which contains all the files which are part of the package
## and are not shown in qtcreator. The createt library is just a non-functional dummy and is
## created in the /tmp folder to avoid confusion.
function (add_files_to_project)
  set(DUMMY_LIB_NAME ${PROJECT_NAME}_stuff)
  message(STATUS "Creating dummy target '${DUMMY_LIB_NAME}'. Its needed for QtCreator only.")
  FILE(GLOB_RECURSE INC_ALL "include/*.h")
  FILE(GLOB_RECURSE SRC_ALL "src/*.h")
  FILE(GLOB_RECURSE PY_ALL "*.py")
  FILE(GLOB_RECURSE YAML_ALL "param/*.yaml")
  FILE(GLOB_RECURSE LAUNCH_ALL "launch/*.launch")
  FILE(GLOB_RECURSE TEST_ALL "launch/*.test")
  FILE(GLOB_RECURSE MSG_ALL "msg/*.msg")
  FILE(GLOB_RECURSE SRV_ALL "srv/*.srv")
  FILE(GLOB_RECURSE ACTION_ALL "action/*.action")
  FILE(GLOB_RECURSE CFG_ALL "cfg/*.cfg")
  FILE(GLOB_RECURSE SCRIPTS_ALL "scripts/*")
  #set output directory to tmp, so the dummy-library will not be found by ROS
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "/tmp/")
  add_library(${DUMMY_LIB_NAME}
              ${INC_ALL} ${CFG_ALL} ${YAML_ALL} ${SCRIPTS_ALL} ${SRC_ALL} ${PY_ALL}
              ${LAUNCH_ALL} ${TEST_ALL} ${MSG_ALL} ${SRV_ALL} ${ACTION_ALL}
  )

  #we need to tell cmake that the 'source-files' are written in C so that cmake accepts them
  SET_TARGET_PROPERTIES(${DUMMY_LIB_NAME} PROPERTIES LINKER_LANGUAGE C)
endfunction(add_files_to_project)
