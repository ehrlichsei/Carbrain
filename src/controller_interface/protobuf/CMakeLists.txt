cmake_minimum_required(VERSION 2.8.3)

message(STATUS "building protocol buffers...")

set(PROTO_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/kitcar-protobufs/brain_controller")
set(PROTO_MSG_VERSION "${CMAKE_CURRENT_BINARY_DIR}/protobuf_msg_version.h")

#scan for .proto files
file(GLOB PROTO_INPUT "${PROTO_SRC_DIR}/*.proto")

#check for compiler
find_program(PROTOC protoc)
set(PROTOC_C_OUT_FLAG --cpp_out)

#protobuf output
set(PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}")
foreach(PROTO_FILE ${PROTO_INPUT})
  message(STATUS "processing protocol buffer ${PROTO_FILE}")
  #get the name of the file without extension
  get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
  #add the generated files
  set(PROTO_GEN ${PROTO_GEN}
    ${PROTO_GEN_DIR}/${PROTO_NAME}.pb.h
    ${PROTO_GEN_DIR}/${PROTO_NAME}.pb.cc)
endforeach(PROTO_FILE ${PROTO_INPUT})

set(PROTO_GEN ${PROTO_GEN} PARENT_SCOPE)
set(PROTO_GEN_DIR ${PROTO_GEN_DIR} PARENT_SCOPE)

#run compiler
add_custom_command(OUTPUT ${PROTO_GEN}
  COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/kitcar-protobufs/brain_controller/*.proto ${PROTO_GEN_DIR} -r
  COMMAND ${PROTOC} -I${PROTO_GEN_DIR} ${PROTO_GEN_DIR}/*.proto ${PROTOC_C_OUT_FLAG} ${PROTO_GEN_DIR}
  DEPENDS ${PROTOC} ${PROTO_INPUT}
  WORKING_DIRECTORY ${PROTO_GEN_DIR}
  )

add_custom_target(protobuild DEPENDS ${PROTO_GEN} ${PROTO_MSG_VERSION})
set_source_files_properties(${PROTO_GEN} PROPERTIES GENERATED TRUE )

#add_custom_target(protobuild ALL DEPENDS ${PROTO_GEN})



###############################################################################
###                        Protobuf Message Version                         ###
###############################################################################

message(STATUS "generating message version header: ${PROTO_MSG_VERSION}")
add_custom_command(OUTPUT ${PROTO_MSG_VERSION}
  COMMAND ../generate_msg_version.sh ${PROTO_MSG_VERSION}
  WORKING_DIRECTORY ${PROTO_SRC_DIR}
  DEPENDS ${PROTO_INPUT} ${PROTO_SRC_DIR}/../generate_msg_version.sh
)
set_source_files_properties(${PROTO_MSG_VERSION} PROPERTIES GENERATED TRUE)
