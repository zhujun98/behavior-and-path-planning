project(behavior_and_path_planning)

cmake_minimum_required (VERSION 3.0)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g")

OPTION(BUILD_TESTS "test suit" ON)

set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(CAR_SOURCES
    ${PROJECT_SOURCE_DIR}/car.cpp
    ${PROJECT_SOURCE_DIR}/car_states.cpp
    ${PROJECT_SOURCE_DIR}/car_transition_states.cpp
    ${PROJECT_INCLUDE_DIR}/trajectory.hpp
    ${PROJECT_INCLUDE_DIR}/map.hpp
    ${PROJECT_INCLUDE_DIR}/jmt.hpp)

set(SOURCES ${PROJECT_SOURCE_DIR}/main.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

enable_testing()
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

set(TARGET run_app)
add_library(car ${CAR_SOURCES})
add_executable(${TARGET} ${SOURCES})

target_link_libraries(${TARGET} car z ssl uv uWS)
