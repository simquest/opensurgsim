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
	"${CMAKE_CURRENT_SOURCE_DIR}"
)

set(EXAMPLE_SOURCES
	main.cpp
)

set(EXAMPLE_HEADERS
)

set(SEVEN_DOF_EXAMPLE_SOURCES
	falcon_7dof_main.cpp
)

set(SEVEN_DOF_EXAMPLE_HEADERS
)

set(LIBS
	NovintDevice
	IdentityPoseDevice
	VisualTestCommon
	SurgSimInput
	SurgSimFramework
	SurgSimDataStructures
	${NOVINTHDALSDK_LIBRARIES}
	${Boost_LIBRARIES}
	${OPENGL_LIBRARIES}
)

surgsim_add_executable(NovintVisualTest
	"${EXAMPLE_SOURCES}" "${EXAMPLE_HEADERS}")
target_link_libraries(NovintVisualTest ${LIBS})

surgsim_add_executable(Novint7DofVisualTest
	"${SEVEN_DOF_EXAMPLE_SOURCES}" "${SEVEN_DOF_EXAMPLE_HEADERS}")
target_link_libraries(Novint7DofVisualTest ${LIBS})

# Put Novint7DofVisualTest into folder "Devices"
set_target_properties(Novint7DofVisualTest PROPERTIES FOLDER "Devices")

# Put NovintVisualTest into folder "Devices"
set_target_properties(NovintVisualTest PROPERTIES FOLDER "Devices")
