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

set(file_id "bAvdlSnfqr5ikadmr6bg7m")
ExternalProject_Add(yaml-cpp
	URL "https://www.assembla.com/spaces/OpenSurgSim/documents/${file_id}/download/yaml-cpp.tar.gz"
	URL_MD5 "6bd2a7b4cc31ad0b65209a8030dda7ed"
	PREFIX yaml-cpp
	CMAKE_ARGS
		-DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
		-DYAML_CPP_BUILD_TOOLS:BOOL=OFF
		-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
)

ExternalProject_Get_Property(yaml-cpp install_dir)

if(MSVC)
	set(YAML_CPP_LIBRARIES
		debug ${install_dir}/lib/libyaml-cppmdd${CMAKE_STATIC_LIBRARY_SUFFIX}
		optimized ${install_dir}/lib/libyaml-cppmd${CMAKE_STATIC_LIBRARY_SUFFIX}
		CACHE INTERNAL "")
else()
	if(BUILD_SHARED_LIBS)
		set(YAML_CPP_LIBRARIES
			"${install_dir}/lib/${CMAKE_SHARED_LIBRARY_PREFIX}yaml-cpp${CMAKE_SHARED_LIBRARY_SUFFIX}"
			CACHE INTERNAL "")
	else()
		set(YAML_CPP_LIBRARIES
			"${install_dir}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}yaml-cpp${CMAKE_STATIC_LIBRARY_SUFFIX}"
			CACHE INTERNAL "")
	endif()
endif()

set(YAML_CPP_INCLUDE_DIR "${install_dir}/include" CACHE INTERNAL "")

