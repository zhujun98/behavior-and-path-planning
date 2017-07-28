project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources
        src/main.cpp
        src/utilities.h
        src/vehicle.cpp
        src/map.cpp
        src/ego.cpp
        src/ego_state.cpp
        src/ego_state_constant_speed.cpp
        src/ego_state_change_lane.cpp
        src/ego_state_follow_traffic.cpp
        src/ego_transition_state.cpp
        src/ego_transition_CS_to_FT.cpp
        src/ego_transition_CL_to_FT.cpp
        src/ego_transition_FT_to_CS.cpp
        src/ego_transition_FT_to_CL.cpp src/path_planner.cpp src/path_planner.h)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
