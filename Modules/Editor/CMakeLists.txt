project(EditDebug)

cmake_policy(SET CMP0079 NEW)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMake)

find_package(Boost 1.70 COMPONENTS program_options REQUIRED)

# Global compiler options
if (MSVC)
	add_compile_options(/MP /permissive-)
	add_compile_definitions(_WINSOCK_DEPRECATED_NO_WARNINGS
	_SILENCE_CXX17_ADAPTOR_TYPEDEFS_DEPRECATION_WARNING)
endif()

set (glew-cmake_BUILD_SHARED OFF CACHE BOOL "")
configure_dependency(glew)

set(IMGUI_IMPL_GL3 ON CACHE BOOL "")
set(IMGUI_WITH_GL3W OFF CACHE BOOL "")

configure_dependency(imgui)
target_compile_definitions(imgui PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLEW)
target_link_libraries(imgui libglew_static)

set(EDITOR_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(Applications)
add_subdirectory(Library)
