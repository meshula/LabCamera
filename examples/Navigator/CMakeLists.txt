
cmake_minimum_required(VERSION 3.11)
project(Navigator VERSION 0.1.0 LANGUAGES C CXX)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (MSVC)
    set (PLATFORM_DEFS
        WIN32_LEAN_AND_MEAN
        _CRT_SECURE_NO_WARNINGS
    )
endif()

if (APPLE)
    set(ST_GFX_DEFS
        IMGUI_IMPL_API=
        SOKOL_METAL
    )
elseif (WIN32)
    set(ST_GFX_DEFS
        IMGUI_IMPL_API=
        SOKOL_GLCORE33
    )
else()
    set(ST_GFX_DEFS
        IMGUI_IMPL_API=
        SOKOL_GLCORE33
    )
endif()

#-------------------------------------------------------------------------------
# tinygizmo
#-------------------------------------------------------------------------------

add_library(tinygizmo STATIC third/tinygizmo/tiny-gizmo.cpp
                             third/tinygizmo/tiny-gizmo.hpp)

target_include_directories(tinygizmo SYSTEM PUBLIC third/tinygizmo)
target_compile_definitions(tinygizmo PRIVATE ${PLATFORM_DEFS})

#-------------------------------------------------------------------------------
# imgui
#-------------------------------------------------------------------------------

set(IMGUI_SRC
    third/imgui/imgui.cpp
    third/imgui/imgui.h
    third/imgui/imgui_demo.cpp
    third/imgui/imgui_draw.cpp
    third/imgui/imgui_internal.h
    third/imgui/imgui_widgets.cpp
    third/ImGui.natvis
)

add_library(imgui STATIC ${IMGUI_SRC})

target_include_directories(imgui SYSTEM PUBLIC third/imgui)

target_compile_definitions(imgui PRIVATE
    ${ST_GFX_DEFS}
    IMGUI_DEFINE_MATH_OPERATORS
    ${PLATFORM_DEFS}
)


#-------------------------------------------------------------------------------
# sokol
#-------------------------------------------------------------------------------

if (APPLE)
    set(SOKOL_SRC
        src/sokol.mm)
else()
    set(SOKOL_SRC
        src/sokol.cc)
endif()

set(SOKOL_HEADERS 
    third/sokol/sokol_app.h
    third/sokol/sokol_args.h
    third/sokol/sokol_audio.h
    third/sokol/sokol_fetch.h
    third/sokol/sokol_gfx.h
    third/sokol/sokol_time.h
    third/sokol/util/sokol_fontstash.h
    third/sokol/util/sokol_gfx_imgui.h
    third/sokol/util/sokol_gl.h
    third/sokol/util/sokol_imgui.h
)

add_library(sokol STATIC ${SOKOL_SRC} ${SOKOL_HEADERS})

target_include_directories(sokol SYSTEM 
    PUBLIC third/sokol third/sokol/util)

target_link_libraries(sokol imgui)
add_dependencies(sokol imgui)

target_compile_definitions(sokol PRIVATE
    ${ST_GFX_DEFS}
    IMGUI_DEFINE_MATH_OPERATORS
    ${PLATFORM_DEFS}
    USE_DBG_UI
)

install(
    TARGETS sokol
    EXPORT pxrTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/sokol
)

#-------------------------------------------------------------------------------
# Navigator
#-------------------------------------------------------------------------------

set(NAVIGATOR_SRC
    src/gizmo.c
    ../../LabCamera.h
    ../../LabCamera.cpp
    src/main.cpp
    src/gizmo.glsl
)

set(NAVIGATOR_SHADERS
    gizmo
)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated_src)

foreach(SHADER ${NAVIGATOR_SHADERS})
    if (WIN32)
        add_custom_command(
            OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/generated_src/${SHADER}_glsl.h
            COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/third/sokol-tools-bin/bin/win32/sokol-shdc.exe --input ${CMAKE_CURRENT_SOURCE_DIR}/src/${SHADER}.glsl --output ${CMAKE_CURRENT_BINARY_DIR}/generated_src/${SHADER}_glsl.h --slang glsl330:hlsl5:metal_macos
            COMMENT "Running sokol-shdc for ${SHADER}.glsl"
        )
    elseif (APPLE)
        add_custom_command(
            OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/generated_src/${SHADER}_glsl.h
            COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/third/sokol-tools-bin/bin/osx/sokol-shdc --input ${CMAKE_CURRENT_SOURCE_DIR}/src/${SHADER}.glsl --output ${CMAKE_CURRENT_BINARY_DIR}/generated_src/${SHADER}_glsl.h --slang glsl330:hlsl5:metal_macos
            COMMENT "Running sokol-shdc for ${SHADER}.glsl"
        )
    else()
        add_custom_command(
            OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/generated_src/${SHADER}_glsl.h
            COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/third/sokol-tools-bin/bin/linux/sokol-shdc --input ${CMAKE_CURRENT_SOURCE_DIR}/src/${SHADER}.glsl --output ${CMAKE_CURRENT_BINARY_DIR}/generated_src/${SHADER}_glsl.h --slang glsl330:hlsl5:metal_macos
            COMMENT "Running sokol-shdc for ${SHADER}.glsl"
        )
    endif()
    list(APPEND NAVIGATOR_SHADERS_SRC ${CMAKE_CURRENT_BINARY_DIR}/generated_src/${SHADER}_glsl.h)
endforeach()

add_custom_target(CompileNavigatorShaderFiles ALL DEPENDS ${PLAYGROUND_SHADERS_SRC})

add_executable(Navigator
    ${NAVIGATOR_SRC}
    ${ST_GFX_SRC}
    ${NAVIGATOR_SHADERS_SRC}
)

add_dependencies(Navigator CompileNavigatorShaderFiles)

target_compile_definitions(Navigator PRIVATE
    ${ST_GFX_DEFS}
    IMGUI_DEFINE_MATH_OPERATORS USE_DBG_UI
    ${PLATFORM_DEFS}
    SOKOL_WIN32_FORCE_MAIN
)

target_include_directories(Navigator SYSTEM
    PRIVATE third/sokol-samples/sapp
    PRIVATE third/sokol-samples/libs
    PRIVATE third/imgui
)

target_include_directories(Navigator
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../..
    PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated_src
    PRIVATE third/imgui/examples
)

set_property(TARGET Navigator PROPERTY CXX_STANDARD 17)
set_property(TARGET Navigator PROPERTY CXX_STANDARD_REQUIRED ON)

if (WIN32)
  set(PLATFORM_LIBS ws2_32 Iphlpapi.lib opengl32.lib)
endif()

target_link_libraries(Navigator
    ${PLATFORM_LIBS}
    imgui
    tinygizmo
    sokol)
    
if (APPLE)
target_link_libraries(Navigator
    "-framework AppKit"
    "-framework CoreGraphics"
    "-framework Foundation"
    "-framework Metal"
    "-framework MetalKit"
    "-framework QuartzCore"
    )
endif()


#-------------------------------------------------------------------------------
# Installer
#-------------------------------------------------------------------------------

install(
    FILES
        resources/fonts/meshula-icons/meshula-icons.ttf
        DESTINATION bin/Navigator_rsrc/fonts/)

install(TARGETS Navigator DESTINATION bin)
