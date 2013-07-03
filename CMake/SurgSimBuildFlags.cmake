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


## Set build flags through CMake.
## Splitting this out removes excessive verbiage from CMakeLists.txt.

# If no build type is specified, default to "Release".
# Note that this does nothing for VS and the like (but no harm either).
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
	set(CMAKE_BUILD_TYPE "Release")
endif("${CMAKE_BUILD_TYPE}" STREQUAL "")

# We always want to use defines from <math.h>.
add_definitions( -D_USE_MATH_DEFINES )

# G++ (C++ compilation) specific settings
if(CMAKE_COMPILER_IS_GNUCXX)
	# default G++ compilation flags
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

	# Enable support for C++0x/C++11 for G++ if available
	include(CheckCXXCompilerFlag)
	check_cxx_compiler_flag(-std=gnu++11 HAVE_FLAG_STD_GNUXX11)
	if(HAVE_FLAG_STD_GNUXX11)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
	else(HAVE_FLAG_STD_GNUXX11)
		check_cxx_compiler_flag(-std=gnu++0x HAVE_FLAG_STD_GNUXX0X)
		if(HAVE_FLAG_STD_GNUXX0X)
			set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
		else(HAVE_FLAG_STD_GNUXX0X)
			message(WARNING "G++ is missing C++0x/C++11 support; trying anyway.")
		endif(HAVE_FLAG_STD_GNUXX0X)
	endif(HAVE_FLAG_STD_GNUXX11)

endif(CMAKE_COMPILER_IS_GNUCXX)

# GCC (C compilation) specific settings
if(CMAKE_COMPILER_IS_GNUCC)  # "CC"?  really?  sigh.
	# default GCC compilation flags
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
endif(CMAKE_COMPILER_IS_GNUCC)

# Visual Studio C/C++ specific settings
if(MSVC)
	# Sanity check the version.  (You can see this problem if you delete
	# the cache file on disk while cmake-gui has the data in memory.)
	if("${MSVC_VERSION}" STREQUAL "")
		message(FATAL_ERROR
			"MSVC_VERSION isn't correctly set; delete the cache and try again!")
	endif("${MSVC_VERSION}" STREQUAL "")

	# default VC++ compilation flags
	add_definitions( -D_CRT_SECURE_NO_WARNINGS )

	# Set the iterator debug level consistently for Debug builds.
	# [Note that you can't add_definitions() conditionally for debug only,
	#  because for VS add_definitions() affects ALL build types in the project.
	#  But this works, and gets put in the proper place in the project.]
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_ITERATOR_DEBUG_LEVEL=2")
	# Enable parallel builds:
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /MP")  # is this needed?
	set(CMAKE_DEBUG_POSTFIX "d")

	# Work around a stupid template argument limitation in VS 2012
	if(MSVC_VERSION EQUAL 1700)
		add_definitions( -D_VARIADIC_MAX=10 )
	endif(MSVC_VERSION EQUAL 1700)
endif(MSVC)

# Settings for clang/LLVM.  May currently be OS X-specific...
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
	set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
 	set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
	set(CMAKE_CXX_FLAGS                "-Wall -std=c++11")
	set(CMAKE_CXX_FLAGS_DEBUG          "-O0 -g")
	set(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
	set(CMAKE_CXX_FLAGS_RELEASE        "-O4 -DNDEBUG")
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

# Windows-specific settings
if(WIN32)
	add_definitions( -D_WIN32_WINNT=0x0501 )  # request compatibility with WinXP
endif(WIN32)

# Linux-specific settings
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	add_definitions( -D_POSIX_C_SOURCE=200809L )  # request POSIX APIs
endif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
