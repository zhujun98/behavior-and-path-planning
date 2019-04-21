find_package(Threads REQUIRED)

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
        ${GTEST_INCLUDE_DIR}
        ${GMOCK_INCLUDE_DIR}
)

target_link_libraries(run_test
    PRIVATE
        car
        ${GTEST_LIBS_DIR}/libgtest.a
        ${GTEST_LIBS_DIR}/libgtest_main.a
        pthread)
add_test(NAME unittest COMMAND run_test)

set(MAP1 data/highway_map.csv)
configure_file(${PROJECT_SOURCE_DIR}/${MAP1} ${CMAKE_CURRENT_BINARY_DIR}/${MAP1} COPYONLY)
