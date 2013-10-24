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
	GroupTests.cpp
	ManagerTests.cpp
	OsgBoxRepresentationTests.cpp
	OsgCameraTests.cpp
	OsgCapsuleRepresentationTests.cpp
	OsgCylinderRepresentationTests.cpp
	OsgGroupTests.cpp
	OsgLogTests.cpp
	OsgLightTests.cpp
	OsgManagerTests.cpp
	OsgMaterialTests.cpp
	OsgMatrixConversionsTests.cpp
	OsgPlaneTests.cpp
	OsgPlaneRepresentationTests.cpp
	OsgQuaternionConversionsTests.cpp
	OsgRenderTargetTests.cpp
	OsgRepresentationTests.cpp
	OsgRigidTransformConversionsTests.cpp
	OsgScreenSpaceQuadTests.cpp
	OsgShaderTests.cpp
	OsgSphereRepresentationTests.cpp
	OsgTextureTests.cpp
	OsgTexture1dTests.cpp
	OsgTexture2dTests.cpp
	OsgTexture3dTests.cpp
	OsgTextureCubeMapTests.cpp
	OsgTextureRectangleTests.cpp
	OsgTextureUniformTests.cpp
	OsgUniformTests.cpp
	OsgUniformBaseTests.cpp
	OsgUniformTypesTests.cpp
	OsgUnitSphereTests.cpp
	OsgVectorConversionsTests.cpp
	OsgVectorFieldRepresentationTests.cpp
	OsgViewTests.cpp
	OsgViewElementTests.cpp
	ViewTests.cpp
	ViewElementTests.cpp
)

set(UNIT_TEST_HEADERS
	MockObjects.h
	MockOsgObjects.h
)

file(COPY Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

set(LIBS SurgSimFramework SurgSimGraphics gtest ${Boost_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} ${OPENTHREADS_LIBRARY})

surgsim_add_unit_tests(SurgSimGraphicsTest)
