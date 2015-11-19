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

#ifndef SURGSIM_FRAMEWORK_COMPONENT_INL_H
#define SURGSIM_FRAMEWORK_COMPONENT_INL_H



namespace SurgSim
{
namespace Framework
{

template <class Target, class Source>
std::shared_ptr<Target> checkAndConvert(std::shared_ptr<Source> incoming, const std::string& expectedTypeName)
{
	SURGSIM_ASSERT(incoming != nullptr) << "Incoming pointer can't be nullptr";
	auto result = std::dynamic_pointer_cast<Target>(incoming);
	SURGSIM_ASSERT(result != nullptr)
			<< "Expected " << expectedTypeName << " but received " << incoming->getClassName() << " which cannot "
			<< "be converted, in component " << incoming->getFullName() << ".";
	return result;
};

}
}

#endif
