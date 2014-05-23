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
set(LIBS ${LIBS} ${Boost_LIBRARIES})

include_directories(
	"${Boost_INCLUDE_DIR}"
)

if(SURGSIM_PRECOMPILED_HEADERS)
	include_directories(
		${EIGEN3_INCLUDE_DIR}
		${OPENTHREADS_INCLUDE_DIR}
		${OSG_INCLUDE_DIR}
		${YAML_CPP_INCLUDE_DIR}
		"PrecompiledHeader"
	)
	
	if(MSVC)
		if(${CMAKE_MAKE_PROGRAM} MATCHES "ninja\\.exe")
			set(COMPILE_DIR "/CMakeFiles/PrecompiledHeader.dir")
		else()
			set(COMPILE_DIR "/$(Configuration)")
		endif()

		# On every following target
		# /Yu: use the precompiled header
		# /Fp: compiled location
		# /FI: inject the header into every compilation
		# /Zm: increase the compiler RAM
		# /Z7: use alternative debugging format (pdb/idb is incompatible with single pch)
		add_compile_options(
			"/YuPrecompiledHeader.h"
			"/Fp${CMAKE_BINARY_DIR}/SurgSim/PrecompiledHeader/${COMPILE_DIR}/PrecompiledHeader.pch"
			"/FIPrecompiledHeader.h"
			"/Zm331"
		)
		add_compile_options($<$<CONFIG:Debug>:/Z7>)
		add_compile_options($<$<CONFIG:RelWithDebInfo>:/Z7>)
	endif()

	add_subdirectory(PrecompiledHeader)
endif()

add_subdirectory(Blocks)
add_subdirectory(Collision)
add_subdirectory(DataStructures)
add_subdirectory(Devices)
add_subdirectory(Framework)
add_subdirectory(Graphics)
add_subdirectory(Input)
add_subdirectory(Math)
add_subdirectory(Physics)
add_subdirectory(Testing)
