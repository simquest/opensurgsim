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

set(CMAKE_ARGS
	-DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
	-DYAML_CPP_BUILD_TOOLS:BOOL=OFF
	-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
)
if(DEFINED CMAKE_BUILD_TYPE)
	LIST(APPEND CMAKE_ARGS -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE})
endif(DEFINED CMAKE_BUILD_TYPE)

ExternalProject_Add(yaml-cpp
	URL "https://github.com/simquest/yaml-cpp/archive/v0.5.1.2.tar.gz"
	URL_MD5 "15c88ddb1a4607ed1df44a036c6feb76"
	PREFIX yaml-cpp
	CMAKE_ARGS ${CMAKE_ARGS}
)

ExternalProject_Get_Property(yaml-cpp install_dir)

if(MSVC)
	add_library(yaml-cpp-lib STATIC IMPORTED GLOBAL)
	set_target_properties(yaml-cpp-lib PROPERTIES IMPORTED_LOCATION_DEBUG
		"${install_dir}/lib/libyaml-cppmdd${CMAKE_STATIC_LIBRARY_SUFFIX}")
	set_target_properties(yaml-cpp-lib PROPERTIES IMPORTED_LOCATION_RELEASE
		"${install_dir}/lib/libyaml-cppmd${CMAKE_STATIC_LIBRARY_SUFFIX}")
	set_target_properties(yaml-cpp-lib PROPERTIES IMPORTED_LOCATION_RELWITHDEBINFO
		"${install_dir}/lib/libyaml-cppmd${CMAKE_STATIC_LIBRARY_SUFFIX}")
	set_target_properties(yaml-cpp-lib PROPERTIES IMPORTED_LOCATION_MINSIZEREL
		"${install_dir}/lib/libyaml-cppmd${CMAKE_STATIC_LIBRARY_SUFFIX}")
		
else()
	if(BUILD_SHARED_LIBS)
		add_library(yaml-cpp-lib SHARED IMPORTED)
		set_target_properties(yaml-cpp-lib PROPERTIES IMPORTED_LOCATION
			"${install_dir}/lib/${CMAKE_SHARED_LIBRARY_PREFIX}yaml-cpp${CMAKE_SHARED_LIBRARY_SUFFIX}")
	else()
		add_library(yaml-cpp-lib STATIC IMPORTED)
		set_target_properties(yaml-cpp-lib PROPERTIES IMPORTED_LOCATION
			"${install_dir}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}yaml-cpp${CMAKE_STATIC_LIBRARY_SUFFIX}")
	endif()
endif()
add_dependencies(yaml-cpp-lib yaml-cpp)

set(YAML_CPP_LIBRARIES "yaml-cpp-lib" CACHE INTERNAL "")
set(YAML_CPP_INCLUDE_DIR "${install_dir}/include" CACHE INTERNAL "")

