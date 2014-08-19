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

ExternalProject_Add(yaml-cpp
	GIT_REPOSITORY https://git.assembla.com/OpenSurgSim.yaml-cpp.git
	GIT_TAG "94911bfbac1225082fbac253c4620abbdf051aad"
	PREFIX yaml-cpp
	CMAKE_ARGS
		-DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
		-DYAML_CPP_BUILD_TOOLS:BOOL=OFF
)

ExternalProject_Get_Property(yaml-cpp install_dir)

if(MSVC)
	set(YAML_CPP_LIBRARIES
		debug ${install_dir}/lib/libyaml-cppmdd${CMAKE_STATIC_LIBRARY_SUFFIX}
		optimized ${install_dir}/lib/libyaml-cppmd${CMAKE_STATIC_LIBRARY_SUFFIX}
		CACHE INTERNAL "")
else()
	set(YAML_CPP_LIBRARIES
		"${install_dir}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}yaml-cpp${CMAKE_STATIC_LIBRARY_SUFFIX}"
		CACHE INTERNAL "")
endif()

set(YAML_CPP_INCLUDE_DIR "${install_dir}/include" CACHE INTERNAL "")

