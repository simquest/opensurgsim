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


find_package(NovintHdalSdk REQUIRED)

link_directories(${Boost_LIBRARY_DIRS})

set(LIBS
	SurgSimDataStructures
	SurgSimFramework
	SurgSimInput
	SurgSimMath
	${Boost_LIBRARIES}
	${NOVINT_HDAL_SDK_LIBRARIES}
	${YAML_CPP_LIBRARIES}
)

include_directories(
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${NOVINT_HDAL_SDK_INCLUDE_DIR}"
)

set(NOVINT_DEVICE_SOURCES
	Novint7DofDevice.cpp
	NovintCommonDevice.cpp
	NovintDevice.cpp
	NovintScaffold.cpp
)

set(NOVINT_DEVICE_HEADERS
	Novint7DofDevice.h
	NovintCommonDevice.h
	NovintDevice.h
	NovintScaffold.h
)

# TODO(advornik): the installation should NOT copy all the headers...
surgsim_add_library(
	NovintDevice
	"${NOVINT_DEVICE_SOURCES}"
	"${NOVINT_DEVICE_HEADERS}"
)

target_link_libraries(NovintDevice ${LIBS})

if(BUILD_TESTING)
	# The unit tests will be built whenever the library is built.
	add_subdirectory(UnitTests)

	if(GLUT_FOUND)
		add_subdirectory(VisualTest)
	endif(GLUT_FOUND)
endif()

# Put NovintDevice into folder "Devices"
set_target_properties(NovintDevice PROPERTIES FOLDER "Devices")
