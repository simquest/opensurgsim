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


link_directories(
	${Boost_LIBRARY_DIRS}
)

include_directories(
	"${SURGSIM_TOP_INCLUDE_DIR}"
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${EIGEN3_INCLUDE_DIR}"
	"${Boost_INCLUDE_DIR}"
	"${GLUT_INCLUDE_DIR}"
)

set(VISUAL_TEST_COMMON_SOURCES
	GlutRenderer.cpp
	MovingSquareForce.cpp
	MovingSquareGlutWindow.cpp
	ToolSquareTest.cpp
)

set(VISUAL_TEST_COMMON_HEADERS
	GlutRenderer.h
	MovingSquareForce.h
	MovingSquareGlutWindow.h
	ToolSquareTest.h
)

# The headers etc. for this do not need to be shipped, so do NOT use
# surgsim_add_library here.
add_library(VisualTestCommon
	${VISUAL_TEST_COMMON_SOURCES} ${VISUAL_TEST_COMMON_HEADERS})
surgsim_show_ide_folders(
	"${VISUAL_TEST_COMMON_SOURCES}" "${VISUAL_TEST_COMMON_HEADERS}")
