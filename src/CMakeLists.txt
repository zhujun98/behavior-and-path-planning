add_library(car STATIC
    car.cpp
    path_optimizer.cpp
    jmt.cpp
    map.cpp
)

target_include_directories(car
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include/behavior-and-path-planning
    PRIVATE
        ${CURRENT_SOURCE_DIR}/
        ${PROJECT_SOURCE_DIR}/third_party/eigen3
        ${PROJECT_SOURCE_DIR}/third_party/spline
)
