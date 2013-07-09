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

#ifndef SURGSIM_TESTING_MATHUTILITIES_H
#define SURGSIM_TESTING_MATHUTILITIES_H


#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Vector.h>

namespace SurgSim
{
namespace Testing
{

	template <class T>
	T lerp(const double& t, const T& start, const T& end)
	{
		return (1-t)*start + t*end;
	}

	SurgSim::Math::RigidTransform3d lerpPose(const double& t, 
										const SurgSim::Math::Vector3d& startAngles, 
										const SurgSim::Math::Vector3d& endAngles,
										const SurgSim::Math::Vector3d& startPosition,
										const SurgSim::Math::Vector3d& endPosition);

} 
}

#endif


