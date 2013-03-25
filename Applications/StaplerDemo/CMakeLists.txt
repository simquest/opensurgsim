set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMake)

## Boost setup
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS thread REQUIRED)
link_directories(${Boost_LIBRARY_DIRS})
set(LIBS ${LIBS} ${Boost_LIBRARIES})

## Need opengl
find_package(OpenGL)
set(LIBS ${LIBS} ${OPENGL_LIBRARIES})

find_package(OpenSceneGraph COMPONENTS osg osgViewer osgText osgUtil osgDB osgGA osgAnimation REQUIRED)
set(LIBS ${LIBS} ${OPENSCENEGRAPH_LIBRARIES})

find_package(OpenThreads)
set(LIBS ${LIBS} ${OPENTHREADS_LIBRARY})

include_directories(
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${Boost_INCLUDE_DIR}"
	"${SURGSIM_TOP_INCLUDE_DIR}"
	"${EIGEN3_INCLUDE_DIR}"
	"${OSG_INCLUDE_DIR}"
)

set(STAPLER_DEMO_SOURCES
	StaplerDemo.cpp
)

set(STAPLER_DEMO_HEADERS
)

# Configure the path for the data files
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/config.txt.in"
	"${CMAKE_CURRENT_BINARY_DIR}/config.txt"
)

add_executable(StaplerDemo ${STAPLER_DEMO_SOURCES} ${STAPLER_DEMO_HEADERS})
target_link_libraries(StaplerDemo ${LIBS} SurgSimFramework SurgSimGraphics SurgSimGraphicsOsg)

# For Visual Studio, group the files by directory.
foreach(file ${STAPLER_DEMO_SOURCES} ${STAPLER_DEMO_HEADERS} ${UNITTEST_SOURCES} ${UNITTEST_HEADERS})
	get_filename_component(file_dir ${file} PATH)
	# Apparently, the hierarchical separator is "\" (per CMake docs).
	string(REGEX REPLACE "/" "\\\\" file_group "${file_dir}")
	if(NOT file_group STREQUAL "")
		source_group("${file_group}" FILES "${file}")
	endif(NOT file_group STREQUAL "")
endforeach(file)
