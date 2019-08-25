cmake_minimum_required(VERSION 3.1)

if(${CMAKE_VERSION} VERSION_LESS 3.13)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
else()
    cmake_policy(VERSION 3.13)
endif()

project(behavior_and_path_planning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(ExternalProject)

OPTION(WITH_BOOST "use boost library" OFF)

if(WITH_BOOST)
    if(${CMAKE_VERSION} VERSION_LESS 3.13)
        add_definitions(-DWITH_BOOST)
    else()
        add_compile_definitions(WITH_BOOST)
    endif()
    set(BOOST_VERSION 1.67.0)
    set(BOOST_ROOT third_party/boost)
    find_package(Boost ${BOOST_VERSION} COMPONENTS log REQUIRED)
endif()

add_subdirectory(src)
add_subdirectory(apps)

OPTION(BUILD_TESTS "test suit" OFF)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()
