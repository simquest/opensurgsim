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

set(SURGSIM_FRAMEWORK_SOURCES
	ApplicationData.cpp
	AssertMessage.cpp
	BasicThread.cpp
	Barrier.cpp
	BehaviorManager.cpp
	Component.cpp
	Logger.cpp
	LogMessageBase.cpp
	LogOutput.cpp
	Representation.cpp
	Runtime.cpp
	Scene.cpp
	SceneElement.cpp
)

set(SURGSIM_FRAMEWORK_HEADERS
	ApplicationData.h
	Assert.h
	AssertMessage.h
	BasicThread.h
	Behavior.h
	BehaviorManager.h
	Component.h
	ComponentManager.h
	LockedContainer.h
	Log.h
	Logger.h
	LogMacros.h
	LogMessage.h
	LogMessageBase.h
	LogOutput.h
	Representation.h
	ReuseFactory.h
	Runtime.h
	Scene.h
	SceneElement.h
	SharedInstance.h
)

surgsim_add_library(
	SurgSimFramework 
	"${SURGSIM_FRAMEWORK_SOURCES}" 
	"${SURGSIM_FRAMEWORK_HEADERS}" 
	"SurgSim/Framework"
)

if(SURGSIM_TESTS_BUILD)
	add_subdirectory(UnitTests)
endif()
