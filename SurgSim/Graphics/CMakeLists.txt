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
	${OSG_INCLUDE_DIR}
	${OPENTHREADS_INCLUDE_DIR}
)

set(SURGSIM_GRAPHICS_SOURCES
	Camera.cpp
	Group.cpp
	Manager.cpp
	Mesh.cpp
	MeshPlyReaderDelegate.cpp
	MeshUtilities.cpp
	OsgAxesRepresentation.cpp
	OsgBoxRepresentation.cpp
	OsgCamera.cpp
	OsgCapsuleRepresentation.cpp
	OsgCylinderRepresentation.cpp
	OsgFont.cpp
	OsgGroup.cpp
	OsgLight.cpp
	OsgLog.cpp
	OsgManager.cpp
	OsgMaterial.cpp
	OsgMeshRepresentation.cpp
	OsgModel.cpp
	OsgOctreeRepresentation.cpp
	OsgPlaneRepresentation.cpp
	OsgPointCloudRepresentation.cpp
	OsgRepresentation.cpp
	OsgSceneryRepresentation.cpp
	OsgScreenSpacePass.cpp
	OsgScreenSpaceQuadRepresentation.cpp
	OsgShader.cpp
	OsgSkeletonRepresentation.cpp
	OsgSphereRepresentation.cpp
	OsgTextRepresentation.cpp
	OsgTexture.cpp
	OsgTexture1d.cpp
	OsgTexture2d.cpp
	OsgTexture3d.cpp
	OsgTextureCubeMap.cpp
	OsgTextureRectangle.cpp
	OsgTrackballZoomManipulator.cpp
	OsgUniformBase.cpp
	OsgUniformFactory.cpp
	OsgUnitAxes.cpp
	OsgVectorFieldRepresentation.cpp
	OsgView.cpp
	OsgViewElement.cpp
	PointCloudRepresentation.cpp
	RenderPass.cpp
	Representation.cpp
	SceneryRepresentation.cpp
	TextRepresentation.cpp
	TriangleNormalGenerator.cpp
	View.cpp
	ViewElement.cpp
)

set(SURGSIM_GRAPHICS_HEADERS
	AxesRepresentation.h
	BoxRepresentation.h
	Camera.h
	CapsuleRepresentation.h
	CylinderRepresentation.h
	Font.h
	Group.h
	Light.h
	Manager.h
	Material.h
	Mesh.h
	Mesh-inl.h
	MeshPlyReaderDelegate.h
	MeshRepresentation.h
	MeshUtilities.h
	Model.h
	OctreeRepresentation.h
	OsgAxesRepresentation.h
	OsgBoxRepresentation.h
	OsgCamera.h
	OsgCapsuleRepresentation.h
	OsgConversions.h
	OsgCylinderRepresentation.h
	OsgFont.h
	OsgGroup.h
	OsgLight.h
	OsgLog.h
	OsgManager.h
	OsgMaterial.h
	OsgMatrixConversions.h
	OsgMeshRepresentation.h
	OsgModel.h
	OsgOctreeRepresentation.h
	OsgPlane.h
	OsgPlaneRepresentation.h
	OsgPointCloudRepresentation.h
	OsgQuaternionConversions.h
	OsgRenderTarget.h
	OsgRenderTarget-inl.h
	OsgRepresentation.h
	OsgRigidTransformConversions.h
	OsgSceneryRepresentation.h
	OsgScreenSpacePass.h
	OsgScreenSpaceQuadRepresentation.h
	OsgShader.h
	OsgSkeletonRepresentation.h
	OsgSphereRepresentation.h
	OsgTextRepresentation.h
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
	OsgUniformFactory.h
	OsgUniformTypes.h
	OsgUnitAxes.h
	OsgUnitBox.h
	OsgUnitCylinder.h
	OsgUnitSphere.h
	OsgVectorConversions.h
	OsgVectorFieldRepresentation.h
	OsgView.h
	OsgViewElement.h
	PlaneRepresentation.h
	PointCloudRepresentation.h
	RenderPass.h
	RenderTarget.h
	Representation.h
	SceneryRepresentation.h
	ScreenSpaceQuadRepresentation.h
	Shader.h
	SkeletonRepresentation.h
	SphereRepresentation.h
	TextRepresentation.h
	Texture.h
	Texture1d.h
	Texture2d.h
	Texture3d.h
	TextureCubeMap.h
	TextureRectangle.h
	TriangleNormalGenerator.h
	Uniform.h
	UniformBase.h
	VectorField.h
	VectorFieldRepresentation.h
	View.h
	ViewElement.h
)
surgsim_create_library_header(Graphics.h "${SURGSIM_GRAPHICS_HEADERS}")

surgsim_add_library(
	SurgSimGraphics
	"${SURGSIM_GRAPHICS_SOURCES}"
	"${SURGSIM_GRAPHICS_HEADERS}"
)

SET(LIBS
	SurgSimDataStructures
	SurgSimFramework
	SurgSimInput
	KeyboardDevice
	MouseDevice
	${OPENSCENEGRAPH_LIBRARIES}
)

target_link_libraries(SurgSimGraphics ${LIBS}
)

if(BUILD_TESTING)
	add_subdirectory(UnitTests)

  if(BUILD_RENDER_TESTING)
		add_subdirectory(RenderTests)
	endif()
endif()

set_target_properties(SurgSimGraphics PROPERTIES FOLDER "Graphics")
