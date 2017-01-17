# This file is a part of the SimQuest OpenSurgSim extension.
# Copyright 2012-2017, SimQuest Solutions Inc.

include_directories(
	${gtest_SOURCE_DIR}/include
)

set(UNIT_TEST_SOURCES
OdeSolverEulerImplicitOpenClTests.cpp
)

set(UNIT_TEST_HEADERS
)

set(LIBS
	SurgSimParallel
	SurgSimDataStructures
	SurgSimFramework
	SurgSimInput
	SurgSimMath
	SurgSimPhysics
)

# file(COPY Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# configure_file(
# 	"${CMAKE_CURRENT_SOURCE_DIR}/config.txt.in"
# 	"${CMAKE_CURRENT_BINARY_DIR}/config.txt"
# )

surgsim_add_unit_tests(SurgSimParallelTest)

set_target_properties(SurgSimParallelTest PROPERTIES FOLDER "Parallel")
