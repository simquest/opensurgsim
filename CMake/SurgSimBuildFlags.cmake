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

cmake_policy(SET CMP0054 NEW)

# If no build type is specified, default to "Release".
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release")
endif()

# We always want to use defines from <math.h>.
if(MSVC)
	add_definitions( -D_USE_MATH_DEFINES )
endif()

# Define our own debug symbol
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DOSS_DEBUG")

option(SURGSIM_WARNINGS_AS_ERRORS "Treat warnings as errors in the compilation process" OFF)

option(SURGSIM_USE_AVX "Utilise AVX extensions, this will be passed onto eigen" OFF)

# G++ (C++ compilation) specific settings
if(CMAKE_COMPILER_IS_GNUCXX)
	# default G++ compilation flags
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall ")
	
	if (SURGSIM_WARNINGS_AS_ERRORS)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
	endif()

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
	
	if (SURGSIM_USE_AVX)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx")
	endif()

endif(CMAKE_COMPILER_IS_GNUCXX)

# GCC (C compilation) specific settings
if(CMAKE_COMPILER_IS_GNUCC)  # "CC"?  really?  sigh.
	# default GCC compilation flags
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
	
	if (SURGSIM_WARNINGS_AS_ERRORS)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
	endif(SURGSIM_WARNINGS_AS_ERRORS)

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
	add_definitions( -D_SCL_SECURE_NO_WARNINGS )
	
	# Set the iterator debug level consistently for Debug builds.
	# [Note that you can't add_definitions() conditionally for debug only,
	#  because for VS add_definitions() affects ALL build types in the project.
	#  But this works, and gets put in the proper place in the project.]
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_ITERATOR_DEBUG_LEVEL=2")
	# Enable parallel builds:
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
	
	if (SURGSIM_WARNINGS_AS_ERRORS)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /WX")
	endif(SURGSIM_WARNINGS_AS_ERRORS)
	
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /MP")  # is this needed?
	set(CMAKE_DEBUG_POSTFIX "d")

	# Work around a stupid template argument limitation in VS 2012
	if(MSVC_VERSION EQUAL 1700)
		add_definitions( -D_VARIADIC_MAX=10 )
	endif(MSVC_VERSION EQUAL 1700)

	if(BUILD_SHARED_LIBS)
		message(FATAL_ERROR "Please turn off BUILD_SHARED_LIBS. Shared libraries on Windows is currently unsupported.")
	endif()
	
	if (SURGSIM_USE_AVX)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX")
	endif()
endif(MSVC)

# Settings for LLVM Clang.
if("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
	if(APPLE)
		set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
		set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
	endif()

	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -stdlib=libc++")
endif()

# Windows-specific settings
if(WIN32)
	add_definitions( -D_WIN32_WINNT=0x0501 )  # request compatibility with WinXP
endif(WIN32)

# Linux-specific settings
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	add_definitions( -D_POSIX_C_SOURCE=200809L )  # request POSIX APIs
endif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")


set(DEFAULT_EIGEN_ALIGNMENT OFF)
# Enable alignement on 64bit systems by default
if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8" OR SURGSIM_USE_AVX)
	set(DEFAULT_EIGEN_ALIGNMENT ON)
endif()

option(EIGEN_ALIGNMENT "Enable alignment in Eigen" ${DEFAULT_EIGEN_ALIGNMENT})

mark_as_advanced(EIGEN_ALIGNMENT)

if(NOT EIGEN_ALIGNMENT)
	add_definitions( -DEIGEN_DONT_ALIGN )
endif()

option(EIGEN_DEBUG "Enable debug assertions in Eigen" ON)
if(NOT EIGEN_DEBUG)
	add_definitions( -DEIGEN_NO_DEBUG)
endif()