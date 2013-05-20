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

find_package(OpenSceneGraph COMPONENTS osg osgViewer osgText osgUtil osgDB osgGA osgAnimation REQUIRED)
set(LIBS ${LIBS} ${OPENSCENEGRAPH_LIBRARIES})

find_package(OpenThreads)
set(LIBS ${LIBS} ${OPENTHREADS_LIBRARY})

include_directories(
	${gtest_SOURCE_DIR}/include
	${gtest_SOURCE_DIR}
	${EIGEN3_INCLUDE_DIR}
	${OSG_INCLUDE_DIR}
	${OPENTHREADS_INCLUDE_DIR}
)

set(UNIT_TEST_SOURCES
	OsgMatrixConversionsTests.cpp
	OsgQuaternionConversionsTests.cpp
	OsgRigidTransformConversionsTests.cpp
	OsgVectorConversionsTests.cpp
)

set(UNIT_TEST_HEADERS
)

include_directories(
)

set(LIBS SurgSimFramework gtest ${Boost_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} ${OPENTHREADS_LIBRARY})


surgsim_add_unit_tests(SurgSimGraphicsTest)
