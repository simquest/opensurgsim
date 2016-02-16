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
SURGSIM_REGISTER(SurgSim::Math::Shape, SurgSim::Math::PlaneShape, PlaneShape);

PlaneShape::PlaneShape()
{
}

PlaneShape::PlaneShape(const PlaneShape& other) :
	Shape(other.getPose())
{
}

int PlaneShape::getType() const
{
	return SHAPE_TYPE_PLANE;
}

double PlaneShape::getVolume() const
{
	return 0.0;
}

SurgSim::Math::Matrix33d PlaneShape::getSecondMomentOfVolume() const
{
	return Matrix33d::Zero();
}

double PlaneShape::getD() const
{
	return 0.0;
}

SurgSim::Math::Vector3d PlaneShape::getNormal() const
{
	return Vector3d(0.0, 1.0, 0.0);
}

bool PlaneShape::isValid() const
{
	return true;
}

std::shared_ptr<Shape> PlaneShape::getCopy() const
{
	return std::make_shared<PlaneShape>(*this);
}

}; // namespace Math
}; // namespace SurgSim
