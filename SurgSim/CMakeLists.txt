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


link_directories(${Boost_LIBRARY_DIRS})

add_subdirectory(Blocks)
add_subdirectory(Collision)
add_subdirectory(DataStructures)
add_subdirectory(Devices)
add_subdirectory(Framework)
add_subdirectory(Graphics)
add_subdirectory(Input)
add_subdirectory(IntervalArithmetic)
add_subdirectory(Math)
add_subdirectory(Particles)
add_subdirectory(Physics)
add_subdirectory(Testing)
