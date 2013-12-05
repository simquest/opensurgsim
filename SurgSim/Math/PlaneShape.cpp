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

#include "SurgSim/Math/PlaneShape.h"

namespace SurgSim
{
namespace Math
{

PlaneShape::PlaneShape()
{
}

int PlaneShape::getType()
{
	return SHAPE_TYPE_PLANE;
}

double PlaneShape::calculateVolume() const
{
	return 0.0;
}

SurgSim::Math::Vector3d PlaneShape::calculateMassCenter() const
{
	return Vector3d(0.0, 0.0, 0.0);
}

SurgSim::Math::Matrix33d PlaneShape::calculateInertia(double rho) const
{
	return Matrix33d::Identity();
}

double PlaneShape::getD() const
{
	return 0.0;
}

SurgSim::Math::Vector3d PlaneShape::getNormal() const
{
	return Vector3d(0.0, 1.0, 0.0);
}

YAML::Node SurgSim::Math::PlaneShape::encode()
{
	return SurgSim::Math::Shape::encode();
}

bool SurgSim::Math::PlaneShape::decode(const YAML::Node& node)
{
	return SurgSim::Math::Shape::decode(node);
}

}; // namespace Math
}; // namespace SurgSim
