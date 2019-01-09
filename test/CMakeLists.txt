find_package(Threads REQUIRED)
find_package(GTest REQUIRED)

set(TEST run_test)
add_executable(${TEST}
    main.cpp
    test_car.cpp
    test_trajectory.cpp
    test_jmt.cpp
    test_map.cpp
    test_utilities.cpp)

target_link_libraries(${TEST} car gtest pthread )
add_test(NAME ${TEST} COMMAND test)