
cmake_minimum_required(VERSION 3.11)
project(Navigator VERSION 0.2.0 LANGUAGES C CXX)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LABCAMERA_IMGUI "Build LabCamera DearImgui binding" OFF)
option(LABCAMERA_BUILD_EXAMPLES "Build LabCamera examples" OFF)

add_library(LabCamera STATIC cpp/include/LabCamera.h cpp/LabCamera.cpp)
target_include_directories(LabCamera PRIVATE cpp/include)
install(
    TARGETS LabCamera
    EXPORT LabCamera
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/LabCamera
)

if (LABCAMERA_IMGUI)
    add_library(LabCameraImGui STATIC cpp/include/LabCameraImGui.h cpp/LabCameraImGui.cpp)

    install(
        TARGETS LabCameraImGui
        EXPORT LabCameraImGui
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        PUBLIC_HEADER DESTINATION include/LabCamera
    )
endif()

if (LABCAMERA_BUILD_EXAMPLES)
    add_subdirectory(examples/Navigator)
endif()
