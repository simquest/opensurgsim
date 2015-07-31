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
	${OSG_INCLUDE_DIR}
	${OPENTHREADS_INCLUDE_DIR}
)

set(SOURCES
	InputVtc.cpp
)

set(HEADERS
)

surgsim_add_executable(InputVtc "${SOURCES}" "${HEADERS}")

set(LIBS 
	IdentityPoseDevice
	SurgSimBlocks
	SurgSimGraphics
	SurgSimMath
	${YAML_CPP_LIBRARIES}
)

if(BUILD_DEVICE_MULTIAXIS)
	list(APPEND LIBS MultiAxisDevice)
endif(BUILD_DEVICE_MULTIAXIS)

if(BUILD_DEVICE_NOVINT)
	list(APPEND LIBS NovintDevice)
endif(BUILD_DEVICE_NOVINT)

if(BUILD_DEVICE_PHANTOM)
	list(APPEND LIBS PhantomDevice)
endif(BUILD_DEVICE_PHANTOM)

if(BUILD_DEVICE_MULTIAXIS)
	list(APPEND LIBS MultiAxisDevice)
endif(BUILD_DEVICE_MULTIAXIS)

if(BUILD_DEVICE_LEAP)
	list(APPEND LIBS LeapDevice)
endif(BUILD_DEVICE_LEAP)

target_link_libraries(InputVtc ${LIBS})

# Put InputVtc into folder "Examples"
set_target_properties(InputVtc PROPERTIES FOLDER "Examples") 
