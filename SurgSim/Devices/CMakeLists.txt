# This file is a part of the OpenSurgSim project.
# Copyright 2012-2013, SimQuest Solutions Inc.
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


add_subdirectory(IdentityPoseDevice)

if(SURGSIM_DEVICES_BUILD)
	if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux" OR WDK_FOUND)
		add_subdirectory(MultiAxis)
	endif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux" OR WDK_FOUND)
	if(NOVINT_HDAL_SDK_FOUND)
		add_subdirectory(Novint)
	endif(NOVINT_HDAL_SDK_FOUND)
	if(OPENHAPTICS_FOUND)
		add_subdirectory(Phantom)
	endif(OPENHAPTICS_FOUND)
	if(SIXENSE_SDK_FOUND)
		add_subdirectory(Sixense)
	endif(SIXENSE_SDK_FOUND)
	if(OPENSCENEGRAPH_FOUND)
		add_subdirectory(Mouse)
	endif(OPENSCENEGRAPH_FOUND)
	if(OPTITRACK_FOUND)
		add_subdirectory(TrackIR)
	endif(OPTITRACK_FOUND)
endif(SURGSIM_DEVICES_BUILD)
