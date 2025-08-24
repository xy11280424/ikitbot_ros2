cmake_minimum_required(VERSION 3.5.1)

include_directories( ../include
        ${EIGEN3_INCLUDE_DIRS})


add_executable(testcross  test.cpp )
target_link_libraries(testcross
        ${catkin_LIBRARIES}
        )
