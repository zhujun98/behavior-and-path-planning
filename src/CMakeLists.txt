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
        ${Boost_INCLUDE_DIR}
)

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)

target_link_libraries(car
    PRIVATE
        pthread
        ${Boost_LIBRARIES}
)
