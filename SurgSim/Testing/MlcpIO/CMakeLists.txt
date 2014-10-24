# This file is a part of the OpenSurgSim project.
# Copyright 2013, SimQuest Solutions Inc.
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
	"${CMAKE_CURRENT_SOURCE_DIR}"
)

set(MLCP_IO_SOURCES
	MlcpTestData.cpp
	ReadText.cpp
	TextLabels.cpp
	WriteText.cpp
)

set(MLCP_IO_HEADERS
	MlcpTestData.h
	ReadText.h
	TextLabels.h
	WriteText.h
)

# The headers etc. for this do not need to be shipped, so do NOT use
# surgsim_add_library here.
add_library(MlcpTestIO
	${MLCP_IO_SOURCES}
	${MLCP_IO_HEADERS}
)
surgsim_show_ide_folders("${MLCP_IO_SOURCES}" "${MLCP_IO_HEADERS}")

SET(LIBS
	SurgSimMath
	SurgSimTesting
)

target_link_libraries(MlcpTestIO ${LIBS})

# Put MlcpTestIO into folder "Testing"
set_target_properties(MlcpTestIO PROPERTIES FOLDER "Testing")
