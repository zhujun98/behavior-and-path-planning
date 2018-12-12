find_package(Threads REQUIRED)
find_package(GTest REQUIRED)

set(TEST run_test)
add_executable(${TEST} test_trajectory.cpp)

target_link_libraries(${TEST} gtest pthread )
