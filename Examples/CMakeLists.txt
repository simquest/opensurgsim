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

if(BUILD_DEVICE_MULTIAXIS)
	add_subdirectory(AddSphereFromInput)
	add_subdirectory(Stapling)
endif(BUILD_DEVICE_MULTIAXIS)

add_subdirectory(InputVtc)

add_subdirectory(DroppingBalls)

add_subdirectory(GraphicsScene)

# Not Required components
find_package(Boost 1.54 COMPONENTS program_options)

if(Boost_PROGRAM_OPTIONS_FOUND)
add_subdirectory(ShowScenery)
else()
message("Can't build example ShowScenery, missing library boost_program_options.")
endif()
