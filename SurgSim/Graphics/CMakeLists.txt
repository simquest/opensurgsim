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
	${EIGEN3_INCLUDE_DIR}
	${OSG_INCLUDE_DIR}
	${OPENTHREADS_INCLUDE_DIR}
)

set(SURGSIM_GRAPHICS_SOURCES
	Group.cpp
	Manager.cpp
	OsgBoxRepresentation.cpp
	OsgCamera.cpp
	OsgCapsuleRepresentation.cpp
	OsgCylinderRepresentation.cpp
	OsgGroup.cpp
	OsgLight.cpp
	OsgLog.cpp
	OsgManager.cpp
	OsgMaterial.cpp
	OsgPlaneRepresentation.cpp
	OsgRepresentation.cpp
	OsgScreenSpaceQuadRepresentation.cpp
	OsgShader.cpp
	OsgSphereRepresentation.cpp
	OsgTexture.cpp
	OsgTexture1d.cpp
	OsgTexture2d.cpp
	OsgTexture3d.cpp
	OsgTextureCubeMap.cpp
	OsgTextureRectangle.cpp
	OsgTrackballZoomManipulator.cpp
	OsgUniformBase.cpp
	OsgView.cpp
	OsgViewElement.cpp
	Representation.cpp
	ViewElement.cpp
)

set(SURGSIM_GRAPHICS_HEADERS
	BoxRepresentation.h
	Camera.h
	CapsuleRepresentation.h
	CylinderRepresentation.h
	Group.h
	Light.h
	Manager.h
	Material.h
	OsgBoxRepresentation.h
	OsgCamera.h
	OsgCapsuleRepresentation.h
	OsgConversions.h
	OsgCylinderRepresentation.h
	OsgGroup.h
	OsgLog.h
	OsgLight.h
	OsgManager.h
	OsgMaterial.h
	OsgMatrixConversions.h
	OsgPlane.h
	OsgPlaneRepresentation.h
	OsgPointCloudRepresentation.h
	OsgPointCloudRepresentation-inl.h
	OsgQuaternionConversions.h
	OsgRenderTarget.h
	OsgRenderTarget-inl.h
	OsgRepresentation.h
	OsgRigidTransformConversions.h
	OsgScreenSpaceQuadRepresentation.h
	OsgShader.h
	OsgSphereRepresentation.h
	OsgTexture.h
	OsgTexture1d.h
	OsgTexture2d.h
	OsgTexture3d.h
	OsgTextureCubeMap.h
	OsgTextureRectangle.h
	OsgTextureUniform.h
	OsgTextureUniform-inl.h
	OsgTrackballZoomManipulator.h
	OsgUniform.h
	OsgUniform-inl.h
	OsgUniformBase.h
	OsgUniformTypes.h
	OsgUnitBox.h
	OsgUnitCylinder.h
	OsgUnitSphere.h
	OsgVectorConversions.h
	OsgView.h
	OsgViewElement.h
	PlaneRepresentation.h
	PointCloudRepresentation.h
	RenderTarget.h
	Representation.h
	ScreenSpaceQuadRepresentation.h
	Shader.h
	SphereRepresentation.h
	Texture.h
	Texture1d.h
	Texture2d.h
	Texture3d.h
	TextureCubeMap.h
	TextureRectangle.h
	Uniform.h
	UniformBase.h
	View.h
	ViewElement.h
)

surgsim_add_library(
	SurgSimGraphics
	"${SURGSIM_GRAPHICS_SOURCES}"
	"${SURGSIM_GRAPHICS_HEADERS}"
	"SurgSim/Graphics"
)
target_link_libraries(SurgSimGraphics SurgSimFramework
	${OPENSCENEGRAPH_LIBRARIES} ${OPENTHREADS_LIBRARY})


if(SURGSIM_TESTS_BUILD)
	add_subdirectory(UnitTests)
	add_subdirectory(RenderTests)
endif()
