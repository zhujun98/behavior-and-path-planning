find_package(Threads REQUIRED)
find_package(GTest REQUIRED)

set(TEST run_test)
add_executable(${TEST}
    ../src/car.cpp
    ../src/car_states.cpp
    ../src/car_transition_states.cpp
    main.cpp
    test_car.cpp
    test_trajectory.cpp
    test_map.cpp
    test_utilities.cpp)

target_link_libraries(${TEST} gtest pthread )
add_test(${TEST} test)