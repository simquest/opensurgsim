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

set(SURGSIM_PARTICLES_SOURCES
	PointGenerator.cpp
	RandomBoxPointGenerator.cpp
	RandomSpherePointGenerator.cpp
)

set(SURGSIM_PARTICLES_HEADERS
	PointGenerator.h
	RandomBoxPointGenerator.h
	RandomSpherePointGenerator.h
)

surgsim_add_library(
	SurgSimParticles
	"${SURGSIM_PARTICLES_SOURCES}"
	"${SURGSIM_PARTICLES_HEADERS}"
)

set_target_properties(SurgSimParticles PROPERTIES FOLDER "Particles")
