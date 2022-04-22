include_directories(${PROJECT_SOURCE_DIR}/include)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

aux_source_directory(. DIR_SRCS)

add_executable(Tracking ${DIR_SRCS})

find_package(osqp REQUIRED)
target_link_libraries(Tracking PRIVATE osqp::osqp)

#target_link_libraries(Tracking Mylib)