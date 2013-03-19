# This file is a part of the OpenSurgSim project.
# Copyright 2013, SimQuest Solutions Inc.
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


link_directories(
	${Boost_LIBRARY_DIRS}
)

include_directories(
	"${SURGSIM_TOP_INCLUDE_DIR}"
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${EIGEN3_INCLUDE_DIR}"
	"${Boost_INCLUDE_DIR}"
	"${GLUT_INCLUDE_DIR}"
)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	# The standard FindGLUT code in CMake 2.8 includes some annoying old junk
	# (-lXmu -lXi) in GLUT_LIBRARIES for Unix/Linux.  We only need GLUT itself.
	set(GLUT_LIBRARIES ${GLUT_glut_LIBRARY})
endif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

set(EXAMPLE_SOURCES
	GlutRenderer.cpp
	main.cpp
	MovingSquareForce.cpp
	MovingSquareGlutWindow.cpp
)

set(EXAMPLE_HEADERS
	GlutRenderer.h
	MovingSquareForce.h
	MovingSquareGlutWindow.h
)

add_executable(MMVR2013_BiggerInputDemo
	${EXAMPLE_SOURCES} ${EXAMPLE_HEADERS})
target_link_libraries(MMVR2013_BiggerInputDemo
	${LIBS} PhantomDevice IdentityPoseDevice SurgSimInput SurgSimFramework SurgSimDataStructures
	${Boost_LIBRARIES} ${OPENHAPTICS_LIBRARIES} ${GLUT_LIBRARIES})
