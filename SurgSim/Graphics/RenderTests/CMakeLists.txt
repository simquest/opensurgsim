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

include_directories(
	${GMOCK_INCLUDE_DIR}
)

set(UNIT_TEST_SOURCES
	OsgBoxRepresentationRenderTests.cpp
	OsgCameraRenderTests.cpp
	OsgCapsuleRepresentationRenderTests.cpp
	OsgCylinderRepresentationRenderTests.cpp
	OsgManagerRenderTests.cpp
	OsgMeshRepresentationRenderTests.cpp
	OsgPlaneRepresentationRenderTests.cpp
	OsgPointCloudRepresentationRenderTests.cpp
	OsgRepresentationRenderTests.cpp
	OsgSceneryRepresentationRenderTests.cpp
	OsgScreenSpaceQuadRenderTests.cpp
	OsgShaderRenderTests.cpp
	OsgSphereRepresentationRenderTests.cpp
	OsgVectorFieldRepresentationRenderTests.cpp
	OsgViewElementRenderTests.cpp
	RenderTest.cpp
)

set(UNIT_TEST_HEADERS
	RenderTest.h
)

file(COPY Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CMAKE_SOURCE_DIR}/SurgSim/Testing/OsgSceneryRepresentationTests DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Data)

set(LIBS
	SurgSimGraphics
	SurgSimBlocks
	SurgSimTesting
	gmock
)

# Configure the path for the data files
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/config.txt.in"
	"${CMAKE_CURRENT_BINARY_DIR}/config.txt"
)

# The unit tests will be built whenever the library is built, but
# running them at build time requires a window system and is disabled by
# default.
option(SURGSIM_TESTS_RUN_GRAPHICS_RENDER
	"Run the rendering tests as part of unit tests (Requires a running window system)"
	OFF)
surgsim_add_unit_tests(SurgSimGraphicsRenderTest
	RUN ${SURGSIM_TESTS_RUN_GRAPHICS_RENDER})

# Put SurgSimGraphicsRenderTest into folder "Graphics"
set_target_properties(SurgSimGraphicsRenderTest PROPERTIES FOLDER "Graphics")
