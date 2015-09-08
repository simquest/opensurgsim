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


# Choose not to link boost::regex as we dont use read_until() or async_read_until()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_REGEX_NO_LIB")

link_directories(${Boost_LIBRARY_DIRS})

set(LIBS
	${Boost_LIBRARIES}
	SurgSimInput
)

include_directories(
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${EIGEN3_INCLUDE_DIR}"
	"${Boost_INCLUDE_DIR}"
)

set(NIMBLE_DEVICE_SOURCES
	NimbleDevice.cpp
	NimbleScaffold.cpp
)

set(NIMBLE_DEVICE_HEADERS
	NimbleDevice.h
	NimbleScaffold.h
)

set(DEVICE_HEADERS ${DEVICE_HEADERS} Nimble/NimbleDevice.h PARENT_SCOPE)

surgsim_add_library(
	NimbleDevice
	"${NIMBLE_DEVICE_SOURCES}"
	"${NIMBLE_DEVICE_HEADERS}"
)

target_link_libraries(NimbleDevice ${LIBS})

if(BUILD_TESTING)
	add_subdirectory(UnitTests)

	if(GLUT_FOUND)
		add_subdirectory(VisualTest)
	endif(GLUT_FOUND)
endif()

# Put NimbleDevice into folder "Nimble"
set_target_properties(NimbleDevice PROPERTIES FOLDER "Devices")
