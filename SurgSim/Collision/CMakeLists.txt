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

set(SURGSIM_COLLISION_SOURCES
	BoxCapsuleDcdContact.cpp
	BoxDoubleSidedPlaneDcdContact.cpp
	BoxPlaneDcdContact.cpp
	BoxSphereDcdContact.cpp
	CapsuleSphereDcdContact.cpp
	CollisionPair.cpp
	ContactCalculation.cpp
	DefaultContactCalculation.cpp
	OctreeDcdContact.cpp
	Representation.cpp
	ShapeCollisionRepresentation.cpp
	SphereDoubleSidedPlaneDcdContact.cpp
	SpherePlaneDcdContact.cpp
	SphereSphereDcdContact.cpp
	TriangleMeshPlaneDcdContact.cpp
	TriangleMeshTriangleMeshDcdContact.cpp
)

set(SURGSIM_COLLISION_HEADERS
	BoxCapsuleDcdContact.h
	BoxDoubleSidedPlaneDcdContact.h
	BoxPlaneDcdContact.h
	BoxSphereDcdContact.h
	CapsuleSphereDcdContact.h
	CollisionPair.h
	ContactCalculation.h
	DcdCollision.h
	DefaultContactCalculation.h
	OctreeDcdContact.h
	Representation.h
	ShapeCollisionRepresentation.h
	SphereDoubleSidedPlaneDcdContact.h
	SpherePlaneDcdContact.h
	SphereSphereDcdContact.h
	TriangleMeshPlaneDcdContact.h
	TriangleMeshTriangleMeshDcdContact.h
)
surgsim_create_library_header(Collision.h "${SURGSIM_COLLISION_HEADERS}")

surgsim_add_library(
	SurgSimCollision
	"${SURGSIM_COLLISION_SOURCES}"
	"${SURGSIM_COLLISION_HEADERS}"
)

SET(LIBS
	SurgSimFramework
	SurgSimMath
	${Boost_LIBRARIES}
)

target_link_libraries(SurgSimCollision ${LIBS}
)

if(BUILD_TESTING)
	add_subdirectory(UnitTests)
endif()

# Put SurgSimCollision into folder "Collision"
set_target_properties(SurgSimCollision PROPERTIES FOLDER "Collision")
