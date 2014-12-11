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


set(SURGSIM_FRAMEWORK_SOURCES
	Accessible.cpp
	ApplicationData.cpp
	AssertMessage.cpp
	Asset.cpp
	Barrier.cpp
	BasicSceneElement.cpp
	BasicThread.cpp
	BehaviorManager.cpp
	Component.cpp
	ComponentManager.cpp
	FrameworkConvert.cpp
	Logger.cpp
	LoggerManager.cpp
	LogMessageBase.cpp
	LogOutput.cpp
	MetricBase.cpp
	PoseComponent.cpp
	Representation.cpp
	Runtime.cpp
	Scene.cpp
	SceneElement.cpp
	Timer.cpp
	TransferPropertiesBehavior.cpp
)

set(SURGSIM_FRAMEWORK_HEADERS
	Accessible.h
	Accessible-inl.h
	ApplicationData.h
	Assert.h
	AssertMessage.h
	Asset.h
	Barrier.h
	BasicSceneElement.h
	BasicThread.h
	Behavior.h
	BehaviorManager.h
	Clock.h
	Component.h
	Component-inl.h
	ComponentManager.h
	ComponentManager-inl.h
	FrameworkConvert.h
	FrameworkConvert-inl.h
	LockedContainer.h
	Log.h
	Logger.h
	LoggerManager.h
	LogMacros.h
	LogMessage.h
	LogMessageBase.h
	LogOutput.h
	Macros.h
	MetricBase.h
	ObjectFactory.h
	ObjectFactory-inl.h
	PoseComponent.h
	Representation.h
	ReuseFactory.h
	Runtime.h
	Scene.h
	SceneElement.h
	SceneElement-inl.h
	SharedInstance.h
	SharedInstance-inl.h
	Timer.h
	TransferPropertiesBehavior.h
)
surgsim_create_library_header(Framework.h "${SURGSIM_FRAMEWORK_HEADERS}")

surgsim_add_library(
	SurgSimFramework
	"${SURGSIM_FRAMEWORK_SOURCES}"
	"${SURGSIM_FRAMEWORK_HEADERS}"
)

SET(LIBS
	${Boost_LIBRARIES}
	${YAML_CPP_LIBRARIES}
)

target_link_libraries(SurgSimFramework ${LIBS})

if(BUILD_TESTING)
	add_subdirectory(UnitTests)
endif()

add_subdirectory(Documentation)


# Put SurgSimFramework into folder "Framework"
set_target_properties(SurgSimFramework PROPERTIES FOLDER "Framework")
