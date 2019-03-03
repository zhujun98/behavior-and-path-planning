add_executable(run_app main.cpp)

target_include_directories(run_app
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/json
        ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(run_app PRIVATE car z ssl uv uWS)
