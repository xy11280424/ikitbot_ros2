find_package(Protobuf REQUIRED)
message(AUTHOR_WARNING "Using protobuf ${Protobuf_VERSION}")

find_package(gRPC REQUIRED)
message(AUTHOR_WARNING "Using gRPC ${gRPC_VERSION}")

find_program(PROTOC protoc)
set(pb_file_root ${CMAKE_CURRENT_SOURCE_DIR})
set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:gRPC::grpc_cpp_plugin>)
set(pb_out_path ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME})
set(grpc_out_path ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME})

function(add_proto_files pb_file_name)
    set(pb_file_path ${pb_file_root})
    set(proto_files ${pb_file_path}/${pb_file_name}.proto)

    # Generated sources
    set(hw_proto_srcs "${pb_out_path}/${pb_file_name}.pb.cc")
    set(hw_proto_hdrs "${pb_out_path}/${pb_file_name}.pb.h")
    set(hw_grpc_srcs "${pb_out_path}/${pb_file_name}.grpc.pb.cc")
    set(hw_grpc_hdrs "${pb_out_path}/${pb_file_name}.grpc.pb.h")

    add_custom_command(
        OUTPUT "${hw_proto_srcs}" "${hw_proto_hdrs}" "${hw_grpc_srcs}" "${hw_grpc_hdrs}"
        COMMAND ${PROTOC}
        ARGS --proto_path=${pb_file_root}
            --grpc_out ${grpc_out_path}
            --cpp_out ${pb_out_path}
            --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
            ${proto_files}
        DEPENDS ${PROTOC} ${proto_files}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
       )

    # Declare a C++ library
    add_library(${pb_file_name}_pb
       ${hw_proto_srcs}
       ${hw_proto_hdrs}
    )

    add_library(${pb_file_name}_grpc_pb
       ${hw_grpc_srcs}
       ${hw_grpc_hdrs}
    )

endfunction()

## Generate protobuf message
add_proto_files(elevator_interface)
## todo install
