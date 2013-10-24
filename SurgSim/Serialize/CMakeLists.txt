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
	${YAML_CPP_SOURCE_DIR}/include
	${OSG_INCLUDE_DIR}
)

set(SURGSIM_SERIALIZE_SOURCES
	FrameworkConvert.cpp
	GraphicsConvert.cpp
	MathConvert.cpp
)

set(SURGSIM_SERIALIZE_HEADERS
	Convert.h
)

surgsim_add_library(
	SurgSimSerialize
	"${SURGSIM_SERIALIZE_SOURCES}"
	"${SURGSIM_SERIALIZE_HEADERS}"
	"SurgSim/Serialize"
)

set(LIBS SurgSimFramework SurgSimGraphics ${Boost_LIBRARIES})


if(SURGSIM_TESTS_BUILD)
	add_subdirectory(UnitTests)
endif()