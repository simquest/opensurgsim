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

include_directories (
	${gtest_SOURCE_DIR}/include
)

set(UNIT_TEST_SOURCES
	BasicSceneElementTests.cpp
	MassSpring1DRepresentationTests.cpp
	MassSpring2DRepresentationTests.cpp
	MassSpring3DRepresentationTests.cpp
	MassSpringNDRepresentationUtilsTests.cpp
	PoseInterpolatorTests.cpp
	SpringTestUtils.cpp
	TransferDeformableStateToVerticesBehaviorTests.cpp
	TransferPoseBehaviorTests.cpp
)

set(UNIT_TEST_HEADERS
	MockObjects.h
	SpringTestUtils.h
)

set(LIBS
	SurgSimBlocks

)

surgsim_add_unit_tests(SurgSimBlocksTest)


set_target_properties(SurgSimBlocksTest PROPERTIES FOLDER "Blocks") 
