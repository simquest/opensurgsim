# Copyright 2016, SimQuest Solutions Inc.

cmake_policy(SET CMP0020 NEW)

FIND_PACKAGE(OpenGL)

set(HEADERS
)

set(SOURCES
	Editor.cpp
)

add_executable(Editor ${SOURCES} ${HEADERS})

#file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

INCLUDE_DIRECTORIES(
    ${OPENGL_INCLUDE_DIR}
	${EDITOR_SOURCE_DIR}
	${GLEW_INCLUDE}
)

configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/config.txt.in"
	"${CMAKE_CURRENT_BINARY_DIR}/config.txt"
)

set(LIBS
	${OPENGL_LIBRARIES}
	SurgSimBlocks
	SurgSimDevices
	SurgSimFramework
	SurgSimGraphics
	SurgSimPhysics
	SurgSimInput
	SurgSimEditDebug
	imgui
	libglew_static
)

target_link_libraries(Editor ${LIBS})
set_target_properties(Editor PROPERTIES FOLDER "EditDebug")
