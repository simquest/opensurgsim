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

#ifndef SURGSIM_PHYSICS_RIGIDREPRESENTATIONBASE_INL_H
#define SURGSIM_PHYSICS_RIGIDREPRESENTATIONBASE_INL_H

namespace SurgSim
{

namespace Physics
{

template <class T>
std::shared_ptr<T> RigidRepresentationBase::createTypedLocalization(
	const SurgSim::DataStructures::Location& location)
{
	// Change when we deal with the meshes as shapes
	std::shared_ptr<T> result = std::make_shared<T>(
		std::static_pointer_cast<Representation>(getSharedPtr()));

	SURGSIM_ASSERT(location.rigidLocalPosition.hasValue()) <<
		"Tried to create a rigid localization without valid position information";

	result->setLocalPosition(location.rigidLocalPosition.getValue());

	return std::move(result);
}

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_RIGIDREPRESENTATIONBASE_INL_H
