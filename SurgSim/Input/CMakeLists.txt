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


set(SURGSIM_INPUT_SOURCES
	CommonDevice.cpp
	InputComponent.cpp
	InputManager.cpp
	OutputComponent.cpp
)

set(SURGSIM_INPUT_HEADERS
	CommonDevice.h
	DeviceInterface.h
	InputComponent.h
	InputConsumerInterface.h
	InputManager.h
	OutputComponent.h
	OutputProducerInterface.h
)
surgsim_create_library_header(Input.h "${SURGSIM_INPUT_HEADERS}")

surgsim_add_library(
	SurgSimInput
	"${SURGSIM_INPUT_SOURCES}"
	"${SURGSIM_INPUT_HEADERS}"
)

set(LIBS  
	SurgSimFramework 
	SurgSimDataStructures 
)

target_link_libraries(SurgSimInput ${LIBS}
)

if(BUILD_TESTING)
	add_subdirectory(UnitTests)
endif()

# Put SurgSimInput into folder "Input"
set_target_properties(SurgSimInput PROPERTIES FOLDER "Input")
