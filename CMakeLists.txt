cmake_minimum_required(VERSION 3.1)

if(${CMAKE_VERSION} VERSION_LESS 3.13)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
else()
    cmake_policy(VERSION 3.13)
endif()

project(behavior_and_path_planning VERSION 2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(BOOST_ROOT third_party/boost)
find_package(Boost 1.67.0 COMPONENTS log REQUIRED)

add_subdirectory(src)
add_subdirectory(apps)

OPTION(BUILD_TESTS "test suit" OFF)

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
