cmake_minimum_required(VERSION 3.5.1)

include_directories( ../include
        ${EIGEN3_INCLUDE_DIRS})


#add_executable(move_to_pose  move_to_pose.cpp )
#target_link_libraries(move_to_pose
#        ${catkin_LIBRARIES}
#        )

add_executable(docker_listener  docker_listener.cpp )
target_link_libraries(docker_listener
        ${catkin_LIBRARIES}
        )
