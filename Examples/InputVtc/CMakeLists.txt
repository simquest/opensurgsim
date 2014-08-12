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
	${CMAKE_CURRENT_SOURCE_DIR}
	${EIGEN3_INCLUDE_DIR}
	${Boost_INCLUDE_DIR}
	${OSG_INCLUDE_DIR}
	${OPENTHREADS_INCLUDE_DIR}
	${YAML_CPP_INCLUDE_DIR}
)

set(INPUTVTCEXAMPLE_SOURCES
	DeviceFactory.cpp
	InputVtcExample.cpp
)

set(INPUTVTCEXAMPLE_HEADERS
	DeviceFactory.h
)

add_executable(ExampleInputVtc
	${INPUTVTCEXAMPLE_SOURCES} ${INPUTVTCEXAMPLE_HEADERS})

set(LIBS 
	IdentityPoseDevice
	SurgSimBlocks
	SurgSimGraphics
	SurgSimMath
	${YAML_CPP_LIBRARIES}
)

if(BUILD_DEVICE_MULTIAXIS)
	list(APPEND LIBS MultiAxisDevice)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMULTIAXISDEVICE_LIBRARY_AVAILABLE")
endif(BUILD_DEVICE_MULTIAXIS)

if(BUILD_DEVICE_NOVINT)
	list(APPEND LIBS NovintDevice)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNOVINT_LIBRARY_AVAILABLE")
endif(BUILD_DEVICE_NOVINT)

target_link_libraries(ExampleInputVtc ${LIBS})

# Put ExampleInputVtc into folder "Examples"
set_target_properties(ExampleInputVtc PROPERTIES FOLDER "Examples") 
