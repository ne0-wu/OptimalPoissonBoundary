cmake_minimum_required(VERSION 3.25)

set(CMAKE_CXX_STANDARD 20)

project(CrossField)

find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenMesh CONFIG REQUIRED)

add_compile_definitions(_USE_MATH_DEFINES)

add_subdirectory(MyGL)

add_executable(CrossField
    main.cpp
    Mesh.h
    Dijkstra.h
    Dijkstra.cpp
    OptimalBoundaries.h
    OptimalBoundaries.cpp
    Parameterization.h
    Parameterization.cpp
)

target_link_libraries(CrossField
    OpenMeshCore
    OpenMeshTools
    MyGL
)

add_custom_command(TARGET CrossField POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/data
        $<TARGET_FILE_DIR:CrossField>/data
    COMMENT "Copying directory to output directory"
)