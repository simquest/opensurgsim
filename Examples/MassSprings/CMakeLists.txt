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

find_package(OpenSceneGraph COMPONENTS osg osgViewer osgText osgUtil osgDB osgGA osgAnimation REQUIRED)
find_package(OpenThreads)

link_directories(
	${Boost_LIBRARY_DIRS}
)

include_directories(
	"${SURGSIM_TOP_INCLUDE_DIR}"
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${EIGEN3_INCLUDE_DIR}"
	"${Boost_INCLUDE_DIR}"
	"${OSG_INCLUDE_DIR}"
	"${OPENTHREADS_INCLUDE_DIR}"
)

set(EXAMPLE_SOURCES
	ExampleMassSprings.cpp
)

set(EXAMPLE_HEADERS
)

add_executable(ExampleMassSprings
	${EXAMPLE_SOURCES} ${EXAMPLE_HEADERS})
surgsim_show_ide_folders(
	"${EXAMPLE_SOURCES}" "${EXAMPLE_HEADERS}")

SET(LIBS
	SurgSimBlocks
	SurgSimPhysics
	SurgSimGraphics
	yaml-cpp
)

target_link_libraries(ExampleMassSprings ${LIBS}
)

# Put ExampleMassSprings into folder "Examples"
set_target_properties(ExampleMassSprings PROPERTIES FOLDER "Examples") 