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



link_directories(${Boost_LIBRARY_DIRS})
set(LIBS ${LIBS} ${Boost_LIBRARIES} ${NOVINT_HDAL_SDK_LIBRARIES})

include_directories(
	"${SURGSIM_TOP_INCLUDE_DIR}"
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${EIGEN3_INCLUDE_DIR}"
	"${Boost_INCLUDE_DIR}"
	"${NOVINT_HDAL_SDK_INCLUDE_DIR}"
)

set(NOVINT_DEVICE_SOURCES
	NovintDevice.cpp
	NovintScaffold.cpp
)

set(NOVINT_DEVICE_HEADERS
	NovintDevice.h
	NovintScaffold.h
)

# TODO(advornik): the installation should NOT copy all the headers...
surgsim_add_library(
	NovintDevice
	"${NOVINT_DEVICE_SOURCES}" "${NOVINT_DEVICE_HEADERS}"
	SurgSim/Devices/Novint
)
if(SURGSIM_TESTS_BUILD)
	# The unit tests will be built whenever the library is built.
	add_subdirectory(UnitTests)

	if(GLUT_FOUND)
		add_subdirectory(VisualTest)
	endif(GLUT_FOUND)
endif()