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

add_library(SurgSimInput ${SURGSIM_INPUT_SOURCES} ${SURGSIM_INPUT_HEADERS})

set_target_properties(SurgSimInput PROPERTIES
	PUBLIC_HEADER "${SURGSIM_INPUT_HEADERS}")

install(TARGETS SurgSimInput
   EXPORT ${PROJECT_NAME}Targets
   RUNTIME DESTINATION bin
   LIBRARY DESTINATION lib
   ARCHIVE DESTINATION lib
   PUBLIC_HEADER DESTINATION include/SurgSim/Input)

if(SURGSIM_TESTS_BUILD)
	add_subdirectory(UnitTests)
endif()
