# Copyright 2016, SimQuest Solutions Inc.

cmake_policy(SET CMP0020 NEW)

FIND_PACKAGE(OpenGL)

set(SOURCES
	EditorManager.h
	imconfig.h
	imgui_internal.h
	ImGuiHandler.h
	stb_rect_pack.h
	stb_textedit.h
	stb_truetype.h
)

set(HEADERS
	Editor.cpp
	EditorManager.cpp
	imgui.cpp
	imgui_draw.cpp
	ImGuiHandler.cpp
)

add_executable(Editor ${SOURCES} ${HEADERS})

#file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

INCLUDE_DIRECTORIES(
    ${OPENGL_INCLUDE_DIR}
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
)

target_link_libraries(Editor ${LIBS})
