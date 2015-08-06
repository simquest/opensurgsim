# This file is a part of the OpenSurgSim project.
# Copyright 2015, SimQuest Solutions Inc.
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

find_package(OculusSdk)

include_directories(
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${OCULUSSDK_INCLUDE_DIR}"
	"${OSG_INCLUDE_DIR}"
)

set(OCULUS_DEVICE_SOURCES
	OculusDevice.cpp
	OculusDisplaySettings.cpp
	OculusScaffold.cpp
	OculusView.cpp
)

set(OCULUS_DEVICE_HEADERS
	OculusDevice.h
	OculusDisplaySettings.h
	OculusScaffold.h
	OculusView.h
)

set(SURGSIM_DEVICES_HEADERS ${SURGSIM_DEVICES_HEADERS} Oculus/OculusDevice.h Oculus/OculusView.h PARENT_SCOPE)

surgsim_add_library(
	OculusDevice
	"${OCULUS_DEVICE_SOURCES}"
	"${OCULUS_DEVICE_HEADERS}"
)

set(LIBS
	SurgSimDataStructures
	SurgSimFramework
	SurgSimGraphics
	SurgSimInput
	${OCULUSSDK_LIBRARY}
	${OPENSCENEGRAPH_LIBRARIES}
)

target_link_libraries(OculusDevice ${LIBS})

if(BUILD_TESTING)
	# The unit tests will be built whenever the library is built.
	add_subdirectory(UnitTests)
	add_subdirectory(SceneTest)

	if(GLUT_FOUND)
		add_subdirectory(VisualTest)
	endif(GLUT_FOUND)
endif()

# Put OculusDevice into folder "Devices"
set_target_properties(OculusDevice PROPERTIES FOLDER "Devices")
