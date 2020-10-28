# This file is a part of the OpenSurgSim project.
# Copyright 2013, SimQuest Solutions Inc.


# - Try to find GLUT via OpenSceneGraph.
#
# Once done this will define
#  GLUT_FOUND - true if the system has GLUT
#  GLUT_INCLUDE_DIR - the GLUT include directory (not including the GL/)
#  GLUT_LIBRARIES - the GLUT libraries
#
# Written from scratch to keep things simple; but does not support OS
# X or Solaris or anything else exciting.  If you need support for
# such things, you may want something like
#
#    find_package(GlutFromOsg)  # this file
#    find_package(GLUT)         # the version that ships with CMake
#
# where several methods will be tried and the first successful one "wins".


find_package(OpenSceneGraph)

if(OPENSCENEGRAPH_FOUND)
	find_path(GLUT_INCLUDE_DIR
		NAMES GL/glut.h
		PATHS ${OSG_INCLUDE_DIR}
		NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH)

	osg_find_library(OSG_GLUT glut32)
	if(OSG_GLUT_LIBRARIES)
		set(GLUT_glut_LIBRARY ${OSG_GLUT_LIBRARY_RELEASE}
			CACHE FILEPATH "GLUT main library." FORCE)
	endif(OSG_GLUT_LIBRARIES)
endif(OPENSCENEGRAPH_FOUND)

include(FindPackageHandleStandardArgs)
# NB: use the same names here as FindGLUT.cmake, so you cam use both
# interchangeably as described in the comments above.  --advornik
find_package_handle_standard_args(GLUT
	DEFAULT_MSG GLUT_glut_LIBRARY GLUT_INCLUDE_DIR)

if(GLUT_FOUND)
	# What if GLUT needs additional libraries to work on this system?
	# (I guess we can figure it out when we run across a system where it
	# does; the GLUT implementations we use on Linux and Windows don't.)
	set(GLUT_LIBRARIES ${GLUT_glut_LIBRARY})
endif(GLUT_FOUND)
mark_as_advanced(GLUT_LIBRARIES)
