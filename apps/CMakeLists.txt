add_executable(run_app main.cpp)

target_include_directories(run_app
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/json
        ${PROJECT_SOURCE_DIR}/include/behavior-and-path-planning
)

target_link_libraries(run_app PRIVATE car z ssl uWS)

set(MAP1 data/highway_map.csv)
set(MAP2 data/highway_map_bosch1.csv)
configure_file(${PROJECT_SOURCE_DIR}/${MAP1} ${CMAKE_CURRENT_BINARY_DIR}/${MAP1} COPYONLY)
configure_file(${PROJECT_SOURCE_DIR}/${MAP2} ${CMAKE_CURRENT_BINARY_DIR}/${MAP2} COPYONLY)
