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

include_directories(
	${EIGEN3_INCLUDE_DIR}
)

set(SURGSIM_GRAPHICS_SOURCES
	Actor.cpp
	ActorImplementation.cpp
	CameraActor.cpp
	CameraElement.cpp
	CameraImplementation.cpp
	Manager.cpp
	Representation.cpp
	Scene.cpp
	SceneImplementation.cpp
	ViewComponent.cpp
	ViewElement.cpp
	ViewImplementation.cpp
)

set(SURGSIM_GRAPHICS_HEADERS
	Actor.h
	ActorImplementation.h
	CameraActor.h
	CameraElement.h
	CameraImplementation.h
	Manager.h
	Representation.h
	Scene.h
	SceneImplementation.h
	ViewComponent.h
	ViewElement.h
	ViewImplementation.h
)

add_library(SurgSimGraphics ${SURGSIM_GRAPHICS_SOURCES} ${SURGSIM_GRAPHICS_HEADERS})

if(SURGSIM_TESTS_BUILD)
#	add_subdirectory(UnitTests)
endif()

add_subdirectory(Osg)
