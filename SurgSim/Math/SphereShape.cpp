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

#include "SurgSim/Math/SphereShape.h"

namespace SurgSim
{
namespace Math
{
SURGSIM_REGISTER(SurgSim::Math::Shape, SurgSim::Math::SphereShape, SphereShape);

SphereShape::SphereShape(double radius) : m_radius(radius)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphereShape, double, Radius, getRadius, setRadius);
	updateAabb();
}

int SphereShape::getType() const
{
	return SHAPE_TYPE_SPHERE;
}

double SphereShape::getRadius() const
{
	return m_radius;
}

void SphereShape::setRadius(double radius)
{
	m_radius = radius;
	updateAabb();
}

void SphereShape::updateAabb()
{
	m_aabb.setEmpty();
	m_aabb.extend(Vector3d(-m_radius, -m_radius, -m_radius));
	m_aabb.extend(Vector3d(m_radius, m_radius, m_radius));
}

double SphereShape::getVolume() const
{
	return 4.0 / 3.0 * M_PI * m_radius * m_radius * m_radius;
}

SurgSim::Math::Vector3d SphereShape::getCenter() const
{
	return Vector3d(0.0, 0.0, 0.0);
}

SurgSim::Math::Matrix33d SphereShape::getSecondMomentOfVolume() const
{
	const double volume = getVolume();

	double diagonalCoefficient = 2.0 / 5.0 * volume * m_radius * m_radius;

	Matrix33d secondMoment;
	secondMoment.setZero();
	secondMoment.diagonal().setConstant(diagonalCoefficient);

	return secondMoment;
}

bool SphereShape::isValid() const
{
	return m_radius >= 0;
}


}; // namespace Math
}; // namespace SurgSim
