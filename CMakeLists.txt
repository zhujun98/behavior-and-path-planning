cmake_minimum_required(VERSION 3.1)

set(GOOGLETEST_VERSION 1.8.1)
set(BOOST_VERSION 1.67.0)

if(${CMAKE_VERSION} VERSION_LESS 3.13)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
else()
    cmake_policy(VERSION 3.13)
endif()

project(behavior_and_path_planning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(BOOST_ROOT third_party/boost)
find_package(Boost ${BOOST_VERSION} COMPONENTS log REQUIRED)

add_subdirectory(src)
add_subdirectory(apps)

OPTION(BUILD_TESTS "test suit" OFF)

if(BUILD_TESTS)
    include(ExternalProject)

    # Download and install GoogleTest
    ExternalProject_Add(
        googletest
        URL https://github.com/google/googletest/archive/release-${GOOGLETEST_VERSION}.zip
        PREFIX ${CMAKE_CURRENT_BINARY_DIR}/googletest
        # Disable install step
        INSTALL_COMMAND ""
        # Wrap download, configure and build steps in a script to log output
        LOG_DOWNLOAD ON
        LOG_CONFIGURE ON
        LOG_BUILD ON
    )

    # Specify include dir
    ExternalProject_Get_Property(googletest source_dir)
    # message(${source_dir})
    set(GTEST_INCLUDE_DIR ${source_dir}/googletest/include)
    set(GMOCK_INCLUDE_DIR ${source_dir}/googlemock/include)

    # Specify link libraries
    ExternalProject_Get_Property(googletest binary_dir)
    set(GTEST_LIBS_DIR ${binary_dir}/googlemock/gtest)

    enable_testing()
    add_subdirectory(test)
endif()
