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

find_package(ATC3DG REQUIRED)

include_directories(
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${ATC3DG_INCLUDE_DIR}"
)

set(LIBS
	SurgSimDataStructures
	SurgSimFramework
	SurgSimInput
	SurgSimMath
	${ATC3DG_LIBRARY}
	${YAML_CPP_LIBRARIES}
)

set(TRAKSTAR_DEVICE_SOURCES
	TrakstarDevice.cpp
	TrakstarScaffold.cpp
)

set(TRAKSTAR_DEVICE_HEADERS
	TrakstarDevice.h
	TrakstarScaffold.h
)

set(DEVICE_HEADERS ${DEVICE_HEADERS} Trakstar/TrakstarDevice.h PARENT_SCOPE)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	message(WARNING "The SDK for trakSTAR does not support Linux.")
else()
	surgsim_add_library(
		TrakstarDevice
		"${TRAKSTAR_DEVICE_SOURCES}"
		"${TRAKSTAR_DEVICE_HEADERS}"
	)

	target_link_libraries(TrakstarDevice ${LIBS})

	if(BUILD_TESTING)
		# The unit tests will be built whenever the library is built.
		add_subdirectory(UnitTests)

		if(GLUT_FOUND)
			add_subdirectory(VisualTest)
		endif(GLUT_FOUND)
	endif()

	# Put TrakstarDevice into folder "Devices"
	set_target_properties(TrakstarDevice PROPERTIES FOLDER "Devices")
endif()
