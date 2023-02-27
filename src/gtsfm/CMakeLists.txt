
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/gtsfm)

add_executable(
    test_exif_read
    test_exif_read.cpp
)
target_link_libraries(
    test_exif_read
    colmap
)

add_executable(
    export_camera_specs
    export_camera_specs.cpp
)
target_link_libraries(
    export_camera_specs
    colmap
)