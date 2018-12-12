find_package(Threads REQUIRED)
find_package(GTest REQUIRED)

set(TEST run_test)
add_executable(${TEST}
    main.cpp
    test_trajectory.cpp
    test_map.cpp)

target_link_libraries(${TEST} gtest pthread )
add_test(${TEST} test)