// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SURGSIM_FRAMEWORK_FRAMEWORKCONVERT_INL_H
#define SURGSIM_FRAMEWORK_FRAMEWORKCONVERT_INL_H

SURGSIM_DOUBLE_SPECIALIZATION
template<typename Base>
static void YAML::convert<std::shared_ptr<SurgSim::Framework::Component>>::registerClass(const std::string& className)
{
	getFactory().registerClass<Base>(className);
}

#endif // SURGSIM_FRAMEWORK_FRAMEWORKCONVERTINL_H