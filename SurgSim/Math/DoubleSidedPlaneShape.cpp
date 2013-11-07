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

#include <SurgSim/Math/DoubleSidedPlaneShape.h>

namespace SurgSim
{
namespace Math
{

DoubleSidedPlaneShape::DoubleSidedPlaneShape()
{
}

int DoubleSidedPlaneShape::getType()
{
	return SHAPE_TYPE_DOUBLESIDEDPLANE;
}

double DoubleSidedPlaneShape::calculateVolume() const
{
	return 0.0;
}

SurgSim::Math::Vector3d DoubleSidedPlaneShape::calculateMassCenter() const
{
	return Vector3d(0.0, 0.0, 0.0);
}

SurgSim::Math::Matrix33d DoubleSidedPlaneShape::calculateInertia(double rho) const
{
	return Matrix33d::Identity();
}

double DoubleSidedPlaneShape::getD() const
{
	return 0.0;
}

SurgSim::Math::Vector3d DoubleSidedPlaneShape::getNormal() const
{
	return Vector3d(0.0, 1.0, 0.0);
}

YAML::Node SurgSim::Math::DoubleSidedPlaneShape::encode()
{
	YAML::Node node;
	node = SurgSim::Math::Shape::encode();
	return node;
}

bool SurgSim::Math::DoubleSidedPlaneShape::decode(const YAML::Node& node)
{
	bool isSuccess = SurgSim::Math::Shape::decode(node);
	if (! isSuccess)
	{
		return false;
	}
	return true;
}
}; // namespace Math
}; // namespace SurgSim
