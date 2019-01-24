add_library(car STATIC
    car.cpp
    path_optimizer.cpp
    jmt.cpp
    trajectory.cpp
    map.hpp
    utilities.hpp)

target_include_directories(car
    PUBLIC
        ${CURRENT_SOURCE_DIR}/
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/eigen3
        ${PROJECT_SOURCE_DIR}/third_party/spline
)

add_executable(run_app main.cpp)

target_include_directories(run_app
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/json
)

target_link_libraries(run_app PRIVATE car z ssl uv uWS)
