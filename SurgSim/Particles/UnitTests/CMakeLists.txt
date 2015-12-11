# This file is a part of the OpenSurgSim project.
# Copyright 2012-2015, SimQuest Solutions Inc.
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
	${gtest_SOURCE_DIR}/include
)

set(UNIT_TEST_SOURCES
	EmitterTests.cpp
	MockObjects.cpp
	ParticlesCollisionRepresentationTests.cpp
	PointGeneratorTests.cpp
	RandomPointGeneratorTests.cpp
	RepresentationTests.cpp
	SinkTests.cpp
	SphRepresentationTests.cpp
)

set(UNIT_TEST_HEADERS
	MockObjects.h
)

configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/config.txt.in"
	"${CMAKE_CURRENT_BINARY_DIR}/config.txt"
)

set(LIBS
	SurgSimCollision
	SurgSimFramework
	SurgSimMath
	SurgSimParticles
	SurgSimPhysics
)

surgsim_add_unit_tests(SurgSimParticlesTest)

set_target_properties(SurgSimParticlesTest PROPERTIES FOLDER "Particles")
