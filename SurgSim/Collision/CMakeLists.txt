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

set(SURGSIM_COLLISION_SOURCES
	BoxCapsuleContact.cpp
	BoxDoubleSidedPlaneContact.cpp
	BoxPlaneContact.cpp
	BoxSphereContact.cpp
	CapsuleSphereContact.cpp
	CollisionPair.cpp
	CompoundShapeContact.cpp
	ContactCalculation.cpp
	ContactFilter.cpp
	DefaultContactCalculation.cpp
	ElementContactFilter.cpp
	OctreeCapsuleContact.cpp
	OctreeContact.cpp
	OctreeDoubleSidedPlaneContact.cpp
	OctreePlaneContact.cpp
	OctreeSphereContact.cpp
	OctreeTriangleMeshContact.cpp
	Representation.cpp
	SegmentMeshTriangleMeshContact.cpp
	SegmentSegmentCcdIntervalCheck.cpp
	SegmentSegmentCcdMovingContact.cpp
	SegmentSegmentCcdStaticContact.cpp
	SegmentSelfContact.cpp
	ShapeCollisionRepresentation.cpp
	SphereDoubleSidedPlaneContact.cpp
	SpherePlaneContact.cpp
	SphereSphereContact.cpp
	TriangleMeshParticlesContact.cpp
	TriangleMeshPlaneContact.cpp
	TriangleMeshSurfaceMeshContact.cpp
	TriangleMeshTriangleMeshContact.cpp
)

set(SURGSIM_COLLISION_HEADERS
	BoxCapsuleContact.h
	BoxDoubleSidedPlaneContact.h
	BoxPlaneContact.h
	BoxSphereContact.h
	CapsuleSphereContact.h
	CcdDcdCollision.h
	CollisionPair.h
	CompoundShapeContact.h
	ContactCalculation.h
	ContactFilter.h
	DefaultContactCalculation.h
	ElementContactFilter.h
	OctreeCapsuleContact.h
	OctreeContact.h
	OctreeDoubleSidedPlaneContact.h
	OctreePlaneContact.h
	OctreeSphereContact.h
	OctreeTriangleMeshContact.h
	Representation.h
	SegmentMeshTriangleMeshContact.h
	SegmentSegmentCcdIntervalCheck.h
	SegmentSegmentCcdMovingContact.h
	SegmentSegmentCcdStaticContact.h
	SegmentSelfContact.h
	ShapeCollisionRepresentation.h
	ShapeShapeContactCalculation.h
	SphereDoubleSidedPlaneContact.h
	SpherePlaneContact.h
	SphereSphereContact.h
	TriangleMeshParticlesContact.h
	TriangleMeshPlaneContact.h
	TriangleMeshSurfaceMeshContact.h
	TriangleMeshTriangleMeshContact.h
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

	if(BUILD_PERFORMANCE_TESTING)
		add_subdirectory(PerformanceTests)
	endif()
endif()

# Put SurgSimCollision into folder "Collision"
set_target_properties(SurgSimCollision PROPERTIES FOLDER "Collision")
