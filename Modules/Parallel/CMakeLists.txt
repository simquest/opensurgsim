# This file is a part of the SimQuest OpenSurgSim extension.
# Copyright 2012-2016, SimQuest Solutions Inc.

set(SURGSIMPARALLEL_SOURCES
	LinearSparseSolveAndInverseOpenCl.cpp
	OdeSolverEulerImplicitLinearOpenCl.cpp
	OdeSolverEulerImplicitOpenCl.cpp
	TimerGroup.cpp
)

set(SURGSIMPARALLEL_HEADERS
	LinearSparseSolveAndInverseOpenCl.h
	OdeSolverEulerImplicitLinearOpenCl.h
	OdeSolverEulerImplicitOpenCl.h
	TimerGroup.h
)

SET(LIBS
	SurgSimCollision
	SurgSimDataStructures
	SurgSimFramework
	SurgSimInput
	SurgSimMath
	SurgSimPhysics
	${Boost_LIBRARIES}
	${YAML_CPP_LIBRARIES}
	${OpenCL_LIBRARIES}
	${VIENNACL_LIBRARIES}
)

if(MSVC)
	if (NOT "${CMAKE_CXX_FLAGS}" MATCHES "/bigobj")
	   set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
	endif()
	if (NOT "${CMAKE_C_FLAGS}" MATCHES "/bigobj")
	   set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /bigobj" )
	endif()
endif()
  
surgsim_add_library(
	SurgSimParallel
	"${SURGSIMPARALLEL_SOURCES}"
	"${SURGSIMPARALLEL_HEADERS}"
	"SurgSim/Physics"
)

target_link_libraries(SurgSimParallel ${LIBS})

set_target_properties(SurgSimParallel PROPERTIES FOLDER "Parallel")

if(BUILD_TESTING)
	add_subdirectory(UnitTests)
	if(BUILD_RENDER_TESTING)
		add_subdirectory(RenderTests)
	endif()
endif()
