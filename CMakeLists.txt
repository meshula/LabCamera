
cmake_minimum_required(VERSION 3.19)
project(Navigator VERSION 0.3.0 LANGUAGES C CXX)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
include(GNUInstallDirs) # for CMAKE_INSTALL_LIBDIR and so on
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS False)

option(LABCAMERA_IMGUI "Build LabCamera DearImgui binding" OFF)
option(LABCAMERA_EXAMPLES "Build LabCamera examples" ON)
set(LABCAMERA_IMGUI_PATH "examples/Navigator/third/imgui" CACHE FILEPATH "path to Dear ImGui")

configure_file(cpp/LabCameraVersion.h.in LabCameraVersion.h)
set(LABCAMERA_PUBLIC_H cpp/include/LabCamera/LabCamera.h)
add_library(LabCamera STATIC ${LABCAMERA_PUBLIC_H} cpp/LabCamera.cpp)
target_include_directories(LabCamera 
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
)
set_target_properties(LabCamera PROPERTIES
    FOLDER "LabCamera"
    POSITION_INDEPENDENT_CODE ON
    PUBLIC_HEADER "${LABCAMERA_PUBLIC_H}"
    OUTPUT_NAME "LabCamera"
    OUTPUT_NAME_DEBUG "LabCamera_d"
)

set(LABCAMERA_INSTALLABLE LabCamera)

configure_file(LabCameraConfig.cmake.in "${PROJECT_BINARY_DIR}/LabCameraConfig.cmake" @ONLY)
add_library(Lab::LabCamera ALIAS LabCamera)

if (LABCAMERA_IMGUI OR LABCAMERA_EXAMPLES)

    set(LABCAMERA_IMGUI_PUBLIC_H cpp/include/LabCamera/LabCameraImGui.h)
    add_library(LabCameraImGui STATIC ${LABCAMERA_IMGUI_PUBLIC_H} cpp/LabCameraImGui.cpp)
    target_include_directories(LabCameraImGui
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include ${LABCAMERA_IMGUI_PATH}>
            $<INSTALL_INTERFACE:include}>
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include ${LABCAMERA_IMGUI_PATH}
    ) 
    set_target_properties(LabCameraImGui PROPERTIES
        FOLDER "LabCameraImGui"
        POSITION_INDEPENDENT_CODE ON
        PUBLIC_HEADER "${LABCAMERA_IMGUI_PUBLIC_H}"
        OUTPUT_NAME "LabCameraImGui"
        OUTPUT_NAME_DEBUG "LabCameraImGui_d"
    )

    list(APPEND LABCAMERA_INSTALLABLE LabCameraImGui)
    add_library(Lab::LabCameraImGui ALIAS LabCameraImGui)
endif()

install(
    TARGETS ${LABCAMERA_INSTALLABLE} 
    EXPORT LabCameraTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} 
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_PREFIX}/include/LabCamera"
)

install(EXPORT LabCameraTargets 
    FILE LabCameraTargets.cmake 
    NAMESPACE Lab::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/LabCamera")

include(CMakePackageConfigHelpers)
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/LabCameraConfigVersion.cmake"
    VERSION "${LabCamera_VERSION_MAJOR}.${LabCamera_VERSION_MINOR}"
    COMPATIBILITY AnyNewerVersion)

configure_package_config_file("${CMAKE_CURRENT_SOURCE_DIR}/LabCameraConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/LabCameraConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/LabCamera"
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

install(FILES 
    "${PROJECT_BINARY_DIR}/LabCameraVersion.h" DESTINATION "${CMAKE_INSTALL_PREFIX}/include/LabCamera")
install(FILES 
    "${CMAKE_CURRENT_BINARY_DIR}/LabCameraConfig.cmake" 
    "${CMAKE_CURRENT_BINARY_DIR}/LabCameraConfigVersion.cmake" 
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/LabCamera")

export(EXPORT LabCameraTargets 
    FILE "${CMAKE_CURRENT_BINARY_DIR}/LabCameraTargets.cmake"
    NAMESPACE Lab::
)

if (LABCAMERA_EXAMPLES)
    add_subdirectory(examples/Navigator)
endif()


