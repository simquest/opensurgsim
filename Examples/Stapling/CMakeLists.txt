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

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	${OSG_INCLUDE_DIR}
)

set(STAPLING_SOURCES
	DeviceFactory.cpp
	Graphics.cpp
	KeyToQuitBehavior.cpp
	StapleElement.cpp
	StaplerBehavior.cpp
	Stapling.cpp
	StaplingPhysicsManager.cpp
)
set(STAPLING_HEADERS
	DeviceFactory.h
	Graphics.h
	KeyToQuitBehavior.cpp
	StapleElement.h
	StaplerBehavior.h
	StaplingPhysicsManager.h
)
surgsim_add_executable(Stapling "${STAPLING_SOURCES}" "${STAPLING_HEADERS}")

set(SERIALIZEDSTAPLING_SOURCES
	SerializedStapling.cpp
	StapleElement.cpp
	StaplerBehavior.cpp
	StaplingPhysicsManager.cpp
)
set(SERIALIZEDSTAPLING_HEADERS
	StapleElement.h
	StaplerBehavior.h
	StaplingPhysicsManager.h
)
surgsim_add_executable(SerializedStapling "${SERIALIZEDSTAPLING_SOURCES}" "${SERIALIZEDSTAPLING_HEADERS}")

set(LIBS 
	IdentityPoseDevice
	MultiAxisDevice
	SurgSimBlocks
	SurgSimDeviceFilters
	SurgSimFramework
	SurgSimGraphics
	${Boost_LIBRARIES}
	${YAML_CPP_LIBRARIES}
)

if(BUILD_DEVICE_PHANTOM)
	list(APPEND LIBS PhantomDevice)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPHANTOM_LIBRARY_AVAILABLE")
endif(BUILD_DEVICE_PHANTOM)

if(BUILD_DEVICE_SIXENSE)
	list(APPEND LIBS SixenseDevice)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSIXENSE_LIBRARY_AVAILABLE")
    surgsim_copy_to_target_directory_for_release(Stapling
        ${SIXENSESDK_sixense_SHARED_RELEASE}
        ${SIXENSESDK_sixense_utils_SHARED_RELEASE}
    )
    surgsim_copy_to_target_directory_for_debug(Stapling
        ${SIXENSESDK_sixense_SHARED_DEBUG}
        ${SIXENSESDK_sixense_utils_SHARED_DEBUG}
        ${SIXENSESDK_DeviceDLL_SHARED_DEBUG}  # crazy debug-only dependency
    )
endif(BUILD_DEVICE_SIXENSE)

if(BUILD_DEVICE_NOVINT)
	list(APPEND LIBS NovintDevice)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNOVINT_LIBRARY_AVAILABLE")
endif(BUILD_DEVICE_NOVINT)

target_link_libraries(Stapling ${LIBS})
target_link_libraries(SerializedStapling ${LIBS})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Configure the path for the data files
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/config.txt.in"
	"${CMAKE_CURRENT_BINARY_DIR}/config.txt"
)

set_target_properties(Stapling PROPERTIES FOLDER "Examples") 
set_target_properties(SerializedStapling PROPERTIES FOLDER "Examples") 
