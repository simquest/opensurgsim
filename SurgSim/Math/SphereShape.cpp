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

#include <SurgSim/Math/SphereShape.h>

namespace SurgSim
{
namespace Math
{

SphereShape::SphereShape(double radius)
{
	m_radius = radius;
}

int SphereShape::getType()
{
	return SHAPE_TYPE_SPHERE;
}

double SphereShape::getRadius() const
{
	return m_radius;
}

double SphereShape::calculateVolume() const
{
	return 4.0 / 3.0 * M_PI * m_radius * m_radius * m_radius;
}

SurgSim::Math::Vector3d SphereShape::calculateMassCenter() const
{
	return Vector3d(0.0, 0.0, 0.0);
}

SurgSim::Math::Matrix33d SphereShape::calculateInertia(double rho) const
{
	const double mass = calculateMass(rho);

	double diagonalCoefficient = 2.0 / 5.0 * mass * m_radius * m_radius;

	Matrix33d inertia;
	inertia.setZero();
	inertia.diagonal().setConstant(diagonalCoefficient);

	return inertia;
}


}; // namespace Math
}; // namespace SurgSim
