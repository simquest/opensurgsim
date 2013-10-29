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

set(SURGSIM_COLLISION_SOURCES
	BoxDoubleSidedPlaneDcdContact.cpp
	BoxPlaneDcdContact.cpp
	BoxSphereDcdContact.cpp
	CapsuleSphereDcdContact.cpp
	CollisionPair.cpp
	CollisionRepresentation.cpp
	ContactCalculation.cpp
	DefaultContactCalculation.cpp
	RigidCollisionRepresentation.cpp
	RigidShapeCollisionRepresentation.cpp
	SphereDoubleSidedPlaneDcdContact.cpp
	SpherePlaneDcdContact.cpp
	SphereSphereDcdContact.cpp
)

set(SURGSIM_COLLISION_HEADERS
	BoxDoubleSidedPlaneDcdContact.h
	BoxPlaneDcdContact.h
	BoxSphereDcdContact.h
	CapsuleSphereDcdContact.h
	CollisionPair.h
	CollisionRepresentation.h
	ContactCalculation.h
	DcdCollision.h
	DefaultContactCalculation.h
	Location.h
	RigidCollisionRepresentation.h
	RigidShapeCollisionRepresentation.h
	SphereDoubleSidedPlaneDcdContact.h
	SpherePlaneDcdContact.h
	SphereSphereDcdContact.h
	TriangleMeshPlaneDcdContact.h
	TriangleMeshPlaneDcdContact-inl.h
)

surgsim_add_library(
	SurgSimCollision
	"${SURGSIM_COLLISION_SOURCES}"
	"${SURGSIM_COLLISION_HEADERS}"
	"SurgSim/Collision"
)
target_link_libraries(SurgSimCollision SurgSimFramework SurgSimMath)

if(SURGSIM_TESTS_BUILD)
	add_subdirectory(UnitTests)
endif()

# Put SurgSimCollision into folder "Collision"
set_target_properties(SurgSimCollision PROPERTIES FOLDER "Collision")