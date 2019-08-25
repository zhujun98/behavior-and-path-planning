# Download and unpack googletest at configure time
configure_file(downloadGTest.cmake.in googletest-download/CMakeLists.txt)
execute_process(
    COMMAND ${CMAKE_COMMAND} -G ${CMAKE_GENERATOR} .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download
)
if (result)
    message(FATAL_ERROR "CMAKE step for gtest failed: ${result}")
endif()

execute_process(
    COMMAND ${CMAKE_COMMAND} --build .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download
)
if (result)
    message(FATAL_ERROR "BUILD step for gtest failed: ${result}")
endif()

# Prevent GoogleTest from overriding our compiler/linker options
# when building with Visual Studio
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This adds
# the following targets: gtest, gtest_main, gmock
# and gmock_main
add_subdirectory("${CMAKE_BINARY_DIR}/googletest-src"
                 "${CMAKE_BINARY_DIR}/googletest-build" EXCLUDE_FROM_ALL)

set(GTEST_INCLUDE_DIRS "${gtest_SOURCE_DIR}/include")
set(GMOCK_INCLUDE_DIRS "${gmock_SOURCE_DIR}/include")

message(STATUS "gtest include directories: ${GTEST_INCLUDE_DIRS}")


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
        ${GTEST_INCLUDE_DIRS}
        ${GMOCK_INCLUDE_DIRS}
)

target_link_libraries(run_test
    PRIVATE
        car
        gtest
        pthread)

add_custom_target(unittest COMMAND run_test DEPENDS run_test)

set(MAP1 data/highway_map.csv)
configure_file(${PROJECT_SOURCE_DIR}/${MAP1} ${CMAKE_CURRENT_BINARY_DIR}/${MAP1} COPYONLY)
