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
	${OSG_INCLUDE_DIR}
)

set(SURGSIM_SERIALIZE_SOURCES
	GraphicsConvert.cpp
	ShapesFactory.cpp
)

set(SURGSIM_SERIALIZE_HEADERS
	GraphicsConvert.h
	ShapesFactory.h
	ShapesFactory-inl.h
)
surgsim_create_library_header(OSS Serialize.h "${SURGSIM_SERIALIZE_HEADERS}")

surgsim_add_library(
	OSS
	SurgSimSerialize
	"${SURGSIM_SERIALIZE_SOURCES}"
	"${SURGSIM_SERIALIZE_HEADERS}"
)

set(LIBS 

)

target_link_libraries(SurgSimSerialize ${LIBS}
)

if(BUILD_TESTING)
	add_subdirectory(UnitTests)
endif()

# Put SurgSimSerialize into folder "Serialize"
set_target_properties(SurgSimSerialize PROPERTIES FOLDER "Serialize")
