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
	${gtest_SOURCE_DIR}/include
	${gtest_SOURCE_DIR}
	${EIGEN3_INCLUDE_DIR}
	${OSG_INCLUDE_DIR}
	${OPENTHREADS_INCLUDE_DIR}
)

set(UNIT_TEST_SOURCES
	OsgBoxRepresentationRenderTests.cpp
	OsgCylinderRepresentationRenderTests.cpp
	OsgManagerRenderTests.cpp
	OsgPlaneRepresentationRenderTests.cpp
	OsgShaderRenderTests.cpp
	OsgSphereRepresentationRenderTests.cpp
	OsgViewElementRenderTests.cpp
)

set(UNIT_TEST_HEADERS
)

file(COPY Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

set(LIBS SurgSimFramework SurgSimGraphics gtest ${Boost_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} ${OPENTHREADS_LIBRARY})

# The unit tests will be built whenever the library is built, but
# running them at build time requires a window system and is disabled by
# default.
option(SURGSIM_TESTS_RUN_GRAPHICS_RENDER
	"Run the rendering tests as part of unit tests (Requires a running window system)"
	OFF)
surgsim_add_unit_tests(SurgSimGraphicsRenderTest
	RUN ${SURGSIM_TESTS_RUN_GRAPHICS_RENDER})
