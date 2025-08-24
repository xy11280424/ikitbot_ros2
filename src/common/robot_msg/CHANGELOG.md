## changelog for package robot msg

*1.3.0 (2021-01-20) glh*
------------------------
*  add ElevatorState.msg
*  update SetDevice.srv
*  add proto support

    usage:

        1. generate proto lib file (/robot_msg/proto/CMakeList.txt):

            `add_proto_files(elevator_interface)`

        2. modify (${PACKAGE}/CMakeList.txt)

            ```
            find_package(Protobuf REQUIRED)
            find_package(gRPC REQUIRED)
            target_link_libraries(${PROJECT_NAME}_node
              ${catkin_LIBRARIES}
              ${PROTOBUF_LIBRARY}
              elevator_interface_pb
              elevator_interface_grpc_pb
            )
            ```

        3. add include file

            ` #include "robot_msg/elevator_interface.pb.h" `
            ` #include "robot_msg/elevator_interface.grpc.pb.h" `

*1.2.0 (2021-02-02) glh*
------------------------
*  add Humidifier.msg
*  add base_diagnostic.msg
*  update ActionState.msg
*  update localizationstate.msg
*  update SetDevice.srv

*1.1.1 (2020-12-22) glh*
------------------------
*  add EraseMap.srv

*1.1.0 (2020-12-17) glh*
------------------------
*  update SetRestrict.srv

*1.0.1 (2020-10-31) glh*
------------------------
*  add Restrict.msg
*  add SetRestrict.msg
*  update SetMap.msg

*1.0.0 (2020-10-20) glh*
------------------------
*  add MapNotify.msg
*  update baseinfo.msg
*  update localizationstate.msg

*0.4.0 (2020-09-02) glh*
------------------------
* add service setDevice 

*0.3.0 (2020-08-24) glh*
------------------------
* add move base feedback define

*0.2.0 (2020-05-22) glh*
------------------------
* add baseinfo, deviceinfo

*0.1.0 (2020-05-20) glh*
------------------------
* first commit
