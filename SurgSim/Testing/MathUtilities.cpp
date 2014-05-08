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

#include "SurgSim/Testing/MathUtilities.h"

#include "SurgSim/Math/Quaternion.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::makeRotationQuaternion;
using SurgSim::Math::makeRigidTransform;


namespace SurgSim
{
namespace Testing
{


SurgSim::Math::RigidTransform3d interpolatePose(
	const Vector3d& startAngles,
	const Vector3d& endAngles,
	const Vector3d& startPosition,
	const Vector3d& endPosition,
	const double& t)
{
	Vector3d angles = interpolate(startAngles, endAngles, t);
	Vector3d position = interpolate(startPosition, endPosition, t);
	return makeRigidTransform<Quaterniond,Vector3d>(
		Quaterniond(makeRotationQuaternion(angles.x(), Vector3d::UnitX().eval()) *
		makeRotationQuaternion(angles.y(), Vector3d::UnitY().eval()) *
		makeRotationQuaternion(angles.z(), Vector3d::UnitZ().eval())),
		position);
}

template <>
SurgSim::Math::Quaterniond interpolate(
	const SurgSim::Math::Quaterniond& start,
	const SurgSim::Math::Quaterniond& end,
	const double& t)
{
	return SurgSim::Math::interpolate(start, end, t);
}

template <>
SurgSim::Math::RigidTransform3d interpolate(
	const SurgSim::Math::RigidTransform3d& start,
	const SurgSim::Math::RigidTransform3d& end,
	const double& t)
{
	return SurgSim::Math::interpolate(start, end, t);
}

}; // Testing
}; // SurgSim
