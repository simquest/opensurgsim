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

surgsim_add_executable(SixenseVisualTest
	"${EXAMPLE_SOURCES}" "${EXAMPLE_HEADERS}")

set(LIBS
	SixenseDevice
	IdentityPoseDevice
	VisualTestCommon
	SurgSimInput
	SurgSimFramework
	SurgSimDataStructures
	${Boost_LIBRARIES}
	${SIXENSESDK_LIBRARIES}
	${OPENGL_LIBRARIES}
)

target_link_libraries(SixenseVisualTest ${LIBS})


# Copy the device DLLs into the build directory
surgsim_copy_to_target_directory_for_release(SixenseVisualTest
	${SIXENSESDK_sixense_SHARED_RELEASE}
	${SIXENSESDK_sixense_utils_SHARED_RELEASE}
)

surgsim_copy_to_target_directory_for_debug(SixenseVisualTest
	${SIXENSESDK_sixense_SHARED_DEBUG}
	${SIXENSESDK_sixense_utils_SHARED_DEBUG}
	${SIXENSESDK_DeviceDLL_SHARED_DEBUG}  # crazy debug-only dependency
)

set_target_properties(SixenseVisualTest PROPERTIES FOLDER "Devices")
