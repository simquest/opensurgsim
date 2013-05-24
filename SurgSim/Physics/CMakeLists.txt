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
	FreeMotion.cpp
	CollisionPair.cpp
	DcdCollision.cpp
	PhysicsManager.cpp
)

set(SURGSIM_PHYSICS_HEADERS
	CollisionPair.h
	CollisionRepresentation.h
	Computation.h
	DcdCollision.h
	FreeMotion.h
	PhysicsManager.h
	PhysicsManager-inl.h
)

add_subdirectory(Actors)

set(SURGSIM_PHYSICS_ALL_SOURCES ${SURGSIM_PHYSICS_SOURCES} ${SURGSIM_PHYSICS_ACTORS_SOURCES})
set(SURGSIM_PHYSICS_ALL_HEADERS ${SURGSIM_PHYSICS_HEADERS} ${SURGSIM_PHYSICS_ACTORS_HEADERS})

surgsim_add_library(
    SurgSimPhysics
    "${SURGSIM_PHYSICS_ALL_SOURCES}"
    "${SURGSIM_PHYSICS_ALL_HEADERS}"
    "SurgSim/Physics"
)

if(SURGSIM_TESTS_BUILD)
	add_subdirectory(UnitTests)
endif()

source_group("Source Files\\Actors" FILES ${SURGSIM_PHYSICS_ACTORS_SOURCES})
source_group("Header Files\\Actors" FILES ${SURGSIM_PHYSICS_ACTORS_HEADERS})

