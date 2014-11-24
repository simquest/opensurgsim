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
	${gmock_SOURCE_DIR}/include
)

set(UNIT_TEST_SOURCES
	GroupTests.cpp
	ManagerTests.cpp
	MeshTests.cpp
	OsgBoxRepresentationTests.cpp
	OsgCameraTests.cpp
	OsgCapsuleRepresentationTests.cpp
	OsgCylinderRepresentationTests.cpp
	OsgGroupTests.cpp
	OsgLightTests.cpp
	OsgLogTests.cpp
	OsgManagerTests.cpp
	OsgMaterialTests.cpp
	OsgMatrixConversionsTests.cpp
	OsgMeshRepresentationTests.cpp
	OsgOctreeRepresentationTests.cpp
	OsgPlaneRepresentationTests.cpp
	OsgPlaneTests.cpp
	OsgPointCloudRepresentationTests.cpp
	OsgQuaternionConversionsTests.cpp
	OsgRenderTargetTests.cpp
	OsgRepresentationTests.cpp
	OsgRigidTransformConversionsTests.cpp
	OsgSceneryRepresentationTests.cpp
	OsgScreenSpaceQuadTests.cpp
	OsgShaderTests.cpp
	OsgSphereRepresentationTests.cpp
	OsgTextRepresentationTests.cpp
	OsgTexture1dTests.cpp
	OsgTexture2dTests.cpp
	OsgTexture3dTests.cpp
	OsgTextureCubeMapTests.cpp
	OsgTextureRectangleTests.cpp
	OsgTextureTests.cpp
	OsgTextureUniformTests.cpp
	OsgUniformBaseTests.cpp
	OsgUniformTests.cpp
	OsgUniformTypesTests.cpp
	OsgUnitSphereTests.cpp
	OsgVectorConversionsTests.cpp
	OsgVectorFieldRepresentationTests.cpp
	OsgViewElementTests.cpp
	OsgViewTests.cpp
	RenderPassTests.cpp
	ViewElementTests.cpp
	ViewTests.cpp
)

set(UNIT_TEST_HEADERS
	MockObjects.h
	MockOsgObjects.h
)

# Configure the path for the data files
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/config.txt.in"
	"${CMAKE_CURRENT_BINARY_DIR}/config.txt"
)

file(COPY Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${SURGSIM_SOURCE_DIR}/SurgSim/Testing/OsgSceneryRepresentationTests DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Data)
file(COPY ${SURGSIM_SOURCE_DIR}/SurgSim/Testing/Data/Geometry DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Data)


set(LIBS 
	SurgSimGraphics
	SurgSimMath
	SurgSimTesting
)

surgsim_add_unit_tests(SurgSimGraphicsTest)

set_target_properties(SurgSimGraphicsTest PROPERTIES FOLDER "Graphics")
