project(behavior_and_path_planning)

OPTION(BUILD_TESTS "test suit" ON)

set(SOURCES main.cpp)

set(CAR_SOURCES
    car.cpp
    car_states.cpp
    jmt.cpp
    trajectory.cpp
)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

set(TARGET run_app)
add_library(car STATIC ${CAR_SOURCES})
add_executable(${TARGET} ${SOURCES})

target_link_libraries(${TARGET} car z ssl uv uWS)
