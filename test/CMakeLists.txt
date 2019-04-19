find_package(Threads REQUIRED)
find_package(GTest REQUIRED)

add_executable(run_test
    main.cpp
    test_car.cpp
    test_path_optimizer.cpp
    test_jmt.cpp
    test_map.cpp
    test_utilities.cpp
)

target_include_directories(run_test
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(run_test PRIVATE car gtest pthread)
add_test(NAME run_test COMMAND test)

set(MAP1 data/highway_map.csv)
configure_file(${PROJECT_SOURCE_DIR}/${MAP1} ${CMAKE_CURRENT_BINARY_DIR}/${MAP1} COPYONLY)
