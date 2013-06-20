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
)

set(SURGSIM_PHYSICS_SOURCES
	CollisionPair.cpp
	ContactCalculation.cpp
	ContactConstraintGeneration.cpp
	Computation.cpp
	DcdCollision.cpp
	FemRepresentationParameters.cpp
	FreeMotion.cpp
	Localization.cpp
	PhysicsManager.cpp
	RigidCollisionRepresentation.cpp
	RigidRepresentation.cpp
	RigidShapeCollisionRepresentation.cpp
	VtcRigidRepresentation.cpp
)

set(SURGSIM_PHYSICS_HEADERS
	BoxShape.h
	CapsuleShape.h
	CollisionPair.h
	CollisionRepresentation.h
	Computation.h
	ContactCalculation.h
	ContactConstraintGeneration.cpp
	CylinderShape.h
	DcdCollision.h
	FemRepresentationParameters.h
	FixedRepresentation.h
	FixedRepresentationLocalization.h
	FreeMotion.h
	Localization.h
	Location.h
	MeshShape.h
	PhysicsManager.h
	PhysicsManagerState.h
	PlaneShape.h
	Representation.h
	RigidCollisionRepresentation.h
	RigidRepresentation.h
	RigidRepresentationBase.h
	RigidRepresentationBaseState.h
	RigidRepresentationLocalization.h
	RigidRepresentationParameters.h
	RigidRepresentationState.h
	RigidShape.h
	RigidShapeCollisionRepresentation.h
	Shapes.h
	SphereShape.h
	VtcRigidParameters.h
	VtcRigidRepresentation.h
)

surgsim_add_library(
    SurgSimPhysics
    "${SURGSIM_PHYSICS_SOURCES}"
    "${SURGSIM_PHYSICS_HEADERS}"
    "SurgSim/Physics"
)
target_link_libraries(SurgSimPhysics SurgSimFramework)

if(SURGSIM_TESTS_BUILD)
	add_subdirectory(UnitTests)
endif()
