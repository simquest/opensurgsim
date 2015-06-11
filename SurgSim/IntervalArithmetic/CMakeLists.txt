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



set(SURGSIM_INTERVALARITHMETIC_SOURCES
	Interval.cpp
)

set(SURGSIM_INTERVALARITHMETIC_HEADERS
	Interval.h
	LinearMotion.h
	PolynomialValues.h
	SqMinMax.h
)
surgsim_create_library_header(IntervalArithmetic.h "${SURGSIM_INTERVALARITHMETIC_HEADERS}")

surgsim_add_library(
	SurgSimIntervalArithmetic
	"${SURGSIM_INTERVALARITHMETIC_SOURCES}"
	"${SURGSIM_INTERVALARITHMETIC_HEADERS}"
)

set(LIBS 
	SurgSimDataStructures
	SurgSimFramework 
	${YAML_CPP_LIBRARIES}
)

target_link_libraries(SurgSimIntervalArithmetic ${LIBS})

if(BUILD_TESTING)
	add_subdirectory(UnitTests)
endif()

# Put SurgSimIntervalArithmetic into folder "IntervalArithmetic"
set_target_properties(SurgSimIntervalArithmetic PROPERTIES FOLDER "IntervalArithmetic")
