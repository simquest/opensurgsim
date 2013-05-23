# This file is a part of the OpenSurgSim project.
# Copyright 2012-2013, SimQuest Solutions Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


## XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX ##
##   HACK: these library settings will NOT work for 64bit       ##
## XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX ##
find_path(MKL_INCLUDE_DIR
	NAMES mkl.h
	PATHS "$ENV{MKL_INCLUDE}" "${MKL_INCLUDE}"
	NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
)
find_path(MKL_INCLUDE_DIR NAMES mkl.h)

find_library(MKL_INTELC_LIBRARY
	NAMES mkl_intel_c
	HINTS "$ENV{MKL_LIB}" "${MKL_LIB}"
	NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
)
find_library(MKL_INTELC_LIBRARY NAMES mkl_intel_c)
find_library(MKL_SEQUENTIAL_LIBRARY
	NAMES mkl_sequential
	HINTS "$ENV{MKL_LIB}" "${MKL_LIB}"
	NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
)
find_library(MKL_SEQUENTIAL_LIBRARY NAMES mkl_sequential)
find_library(MKL_CORE_LIBRARY
	NAMES mkl_core
	HINTS "$ENV{MKL_LIB}" "${MKL_LIB}"
	NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH
)
find_library(MKL_CORE_LIBRARY NAMES mkl_core)

if(MKL_INTELC_LIBRARY AND MKL_SEQUENTIAL_LIBRARY AND MKL_CORE_LIBRARY)
	set(MKL_LIBRARIES
		${MKL_INTELC_LIBRARY} ${MKL_SEQUENTIAL_LIBRARY} ${MKL_CORE_LIBRARY})
endif()

if(MSVC)
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SAFESEH:NO")
endif(MSVC)
## XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX ##

include_directories(
	${gtest_SOURCE_DIR}/include
	${gtest_SOURCE_DIR}
	${MKL_INCLUDE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}
)

set(UNIT_TEST_SOURCES
	GeometryTests.cpp
	MakeRigidTransformTests.cpp
	MlcpGaussSeidelSolverTests.cpp
	MlcpTestProblem1.cpp
	ValidTests.cpp
)

set(UNIT_TEST_HEADERS
	MlcpTestData.h
	MlcpTestProblems.h
)

set(UNIT_TEST_EIGEN_TYPE_SOURCES
	MatrixTests.cpp
	QuaternionTests.cpp
	RigidTransformTests.cpp
	VectorTests.cpp
)

set(UNIT_TEST_EIGEN_TYPE_HEADERS
)

option(SURGSIM_TESTS_BUILD_EIGEN
	"Include basic tests for Eigen math class typedefs, which build very slowly."
	OFF)
if(SURGSIM_TESTS_BUILD_EIGEN)
	set(UNIT_TEST_SOURCES ${UNIT_TEST_SOURCES} ${UNIT_TEST_EIGEN_TYPE_SOURCES})
	set(UNIT_TEST_HEADERS ${UNIT_TEST_HEADERS} ${UNIT_TEST_EIGEN_TYPE_HEADERS})

	# Fix a problem with 64-bit builds running out of sections.
	if(MSVC AND CMAKE_CL_64)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
	endif(MSVC AND CMAKE_CL_64)
endif(SURGSIM_TESTS_BUILD_EIGEN)

#set(LIBS SurgSimMath SurgSimFramework ${MKL_LIBRARIES} gtest)
set(LIBS SurgSimFramework ${MKL_LIBRARIES} gtest)

surgsim_add_unit_tests(SurgSimMathTest)
