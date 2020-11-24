include_directories(
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${OPENSURGSIM_INCLUDE_DIRS}"
	"${EIGEN3_INCLUDE_DIR}"
	"${Boost_INCLUDE_DIR}"
	"${OSG_INCLUDE_DIR}"
)

set(EDITDEBUGLIBRARY_SOURCES
	DrawFunctions.cpp
	GlewInitOperation.cpp
	GlewInitWrapper.cpp
	OsgImGuiHandler.cpp
	VisualDebugBehavior.cpp
)

set(EDITDEBUGLIBRARY_HEADERS
	DrawFunctions.h
	GlewInitOperation.h
	GlewInitWrapper.h
	OsgImGuiHandler.h
	VisualDebugBehavior.h	
)

surgsim_add_library(
	SurgSimEditDebug
	"${EDITDEBUGLIBRARY_SOURCES}" 
	"${EDITDEBUGLIBRARY_HEADERS}"
	Library/SurgSimEditDebug
)

set(LIBS
	SurgSimFramework
	SurgSimGraphics
	imgui
	libglew_static
)

target_link_libraries(SurgSimEditDebug ${LIBS})
target_compile_definitions(SurgSimEditDebug PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLEW)
set_target_properties(SurgSimEditDebug PROPERTIES FOLDER "EditDebug")

#if (BUILD_TESTING)
# 	add_subdirectory(UnitTests)
#endif (BUILD_TESTING)
#
#if(BUILD_TESTING AND BUILD_RENDER_TESTING)
#	add_subdirectory(RenderTests)
#endif(BUILD_TESTING AND BUILD_RENDER_TESTING)
