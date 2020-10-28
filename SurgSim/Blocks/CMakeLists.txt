# This file is a part of the OpenSurgSim project.
# Copyright 2012-2016, SimQuest Solutions Inc.
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
	${OPENTHREADS_INCLUDE_DIR}
)

set(SURGSIM_BLOCKS_SOURCES
	CompoundShapeToGraphics.cpp
	DebugDumpBehavior.cpp
	DriveElementFromInputBehavior.cpp
	FunctionBehavior.cpp
	GraphicsUtilities.cpp
	ImplicitSurface.cpp
	KeyBehavior.cpp
	KeyboardCallbackBehavior.cpp
	KeyboardTogglesComponentBehavior.cpp
	KnotIdentificationBehavior.cpp
	MassSpring1DRepresentation.cpp
	MassSpring2DRepresentation.cpp
	MassSpring3DRepresentation.cpp
	PoseInterpolator.cpp
	PostprocessingView.cpp
	ShadowMapping.cpp
	SingleKeyBehavior.cpp
	SphereElement.cpp
	TransferParticlesToPointCloudBehavior.cpp
	TransferPhysicsToGraphicsMeshBehavior.cpp
	TransferPhysicsToPointCloudBehavior.cpp
	TransferPhysicsToVerticesBehavior.cpp
	VisualizeConstraints.cpp
	VisualizeContactsBehavior.cpp
)

set(SURGSIM_BLOCKS_HEADERS
	CompoundShapeToGraphics.h
	DebugDumpBehavior.h
	DriveElementFromInputBehavior.h
	FunctionBehavior.h
	GraphicsUtilities.h
	ImplicitSurface.h
	KeyBehavior.h
	KeyboardCallbackBehavior.h
	KeyboardTogglesComponentBehavior.h
	KnotIdentificationBehavior.h
	MassSpring1DRepresentation.h
	MassSpring2DRepresentation.h
	MassSpring3DRepresentation.h
	PoseInterpolator.h
	PostprocessingView.cpp
	ShadowMapping.h
	SingleKeyBehavior.h
	SphereElement.h
	TransferParticlesToPointCloudBehavior.h
	TransferPhysicsToGraphicsMeshBehavior.h
	TransferPhysicsToPointCloudBehavior.h
	TransferPhysicsToVerticesBehavior.h
	VisualizeConstraints.h
	VisualizeContactsBehavior.h
)
surgsim_create_library_header(Blocks.h "${SURGSIM_BLOCKS_HEADERS}")

surgsim_add_library(
	SurgSimBlocks
	"${SURGSIM_BLOCKS_SOURCES}"
	"${SURGSIM_BLOCKS_HEADERS}"
)

SET(LIBS
	SurgSimFramework
	SurgSimParticles
	SurgSimPhysics
	${Boost_LIBRARIES}
)

target_link_libraries(SurgSimBlocks ${LIBS}
)

if(BUILD_TESTING)
	add_subdirectory(UnitTests)
endif()

if(BUILD_RENDER_TESTING)
	add_subdirectory(RenderTests)
endif()

# Put SurgSimBlocks into folder "Blocks"
set_target_properties(SurgSimBlocks PROPERTIES FOLDER "Blocks")
