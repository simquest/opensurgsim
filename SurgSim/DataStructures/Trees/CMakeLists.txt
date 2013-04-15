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

set(SURGSIM_DATA_STRUCTURES_TREES_SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/Tree.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/TreeData.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/TreeNode.cpp
	PARENT_SCOPE
)

set(SURGSIM_DATA_STRUCTURES_TREES_HEADERS
	${CMAKE_CURRENT_SOURCE_DIR}/Tree.h
	${CMAKE_CURRENT_SOURCE_DIR}/TreeData.h
	${CMAKE_CURRENT_SOURCE_DIR}/TreeNode.h
	PARENT_SCOPE
)

if(SURGSIM_TESTS_BUILD)
#	add_subdirectory(UnitTests)
endif()




