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
find_package(OpenThreads)

include_directories(
	${EIGEN3_INCLUDE_DIR}
	${OSG_INCLUDE_DIR}
	${OPENTHREADS_INCLUDE_DIR}
)

set(SURGSIM_GRAPHICS_SOURCES
	Group.cpp
	Manager.cpp
	OsgCamera.cpp
	OsgGroup.cpp
	OsgManager.cpp
	OsgPlaneRepresentation.cpp
	OsgSphereRepresentation.cpp
	OsgUniform.cpp
	OsgView.cpp
	OsgViewElement.cpp
	ViewElement.cpp
)

set(SURGSIM_GRAPHICS_HEADERS
	Camera.h
	Group.h
	Manager.h
	OsgCamera.h
	OsgConversions.h
	OsgGroup.h
	OsgManager.h
	OsgMatrixConversions.h
	OsgPlane.h
	OsgPlaneRepresentation.h
	OsgQuaternionConversions.h
	OsgRepresentation.h
	OsgRigidTransformConversions.h
	OsgSphereRepresentation.h
	OsgUniform.h
	OsgUniform-inl.h
	OsgUniformTypes.h
	OsgUnitSphere.h
	OsgVectorConversions.h
	OsgView.h
	OsgViewElement.h
	PlaneRepresentation.h
	Representation.h
	SphereRepresentation.h
	Uniform.h
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
