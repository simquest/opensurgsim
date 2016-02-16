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

#include "SurgSim/Math/DoubleSidedPlaneShape.h"

namespace SurgSim
{
namespace Math
{
SURGSIM_REGISTER(SurgSim::Math::Shape, SurgSim::Math::DoubleSidedPlaneShape, DoubleSidedPlaneShape);

DoubleSidedPlaneShape::DoubleSidedPlaneShape()
{
}

DoubleSidedPlaneShape::DoubleSidedPlaneShape(const DoubleSidedPlaneShape& other) :
	Shape(other.getPose())
{
}

int DoubleSidedPlaneShape::getType() const
{
	return SHAPE_TYPE_DOUBLESIDEDPLANE;
}

double DoubleSidedPlaneShape::getVolume() const
{
	return 0.0;
}

SurgSim::Math::Matrix33d DoubleSidedPlaneShape::getSecondMomentOfVolume() const
{
	return Matrix33d::Zero();
}

double DoubleSidedPlaneShape::getD() const
{
	return 0.0;
}

SurgSim::Math::Vector3d DoubleSidedPlaneShape::getNormal() const
{
	return Vector3d(0.0, 1.0, 0.0);
}

bool DoubleSidedPlaneShape::isValid() const
{
	return true;
}

std::shared_ptr<Shape> DoubleSidedPlaneShape::getCopy() const
{
	return std::make_shared<DoubleSidedPlaneShape>(*this);
}

}; // namespace Math
}; // namespace SurgSim
