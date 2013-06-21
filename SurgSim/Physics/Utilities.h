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

/// \file
/// Helper functions that don't necessarily need to be class members

#ifndef SURGSIM_PHYSICS_UTILITIES_H
#define SURGSIM_PHYSICS_UTILITIES_H

#include <SurgSim/Physics/Representation.h>
#include <SurgSim/Physics/Location.h>
#include <SurgSim/Physics/Localization.h>
#include <SurgSim/Math/RigidTransform.h>

namespace SurgSim
{
namespace Physics
{
/// Creates typed localization.
/// \tparam	T	Type of localization to create.
/// \param	location	The location for the localization.
/// \return	The new Localization;
template <class T>
std::shared_ptr<T> createTypedLocalization(const Representation& representation, const Location& location)
{
	// Change when we deal with the meshes as shapes
	std::shared_ptr<T> result = std::make_shared<T>();

	SURGSIM_ASSERT(location.globalPosition.hasValue() || location.rigidLocalPosition.hasValue()) <<
		"Tried to create a rigid localization without valid position information";

	SurgSim::Math::Vector3d localPosition;
	if (!location.rigidLocalPosition.hasValue())
	{
		//localPosition = representation.getCurrentPose().inverse() * location.globalPosition.getValue();
	}
	else
	{
		localPosition = location.rigidLocalPosition.getValue();
	}

	result->setLocalPosition(localPosition);

	return std::move(result);
}

}; // Physics
}; // SurgSim

#endif