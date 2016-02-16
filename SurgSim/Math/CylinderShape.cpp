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

#include "SurgSim/Math/CylinderShape.h"

namespace SurgSim
{
namespace Math
{
SURGSIM_REGISTER(SurgSim::Math::Shape, SurgSim::Math::CylinderShape, CylinderShape);

CylinderShape::CylinderShape(double length, double radius) : m_length(length), m_radius(radius)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(CylinderShape, double, Radius, getRadius, setRadius);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(CylinderShape, double, Length, getLength, setLength);
	updateAabb();
}

CylinderShape::CylinderShape(const CylinderShape& other) :
	Shape(other.getPose()),
	m_length(other.getLength()),
	m_radius(other.getRadius())
{
	updateAabb();
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(CylinderShape, double, Radius, getRadius, setRadius);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(CylinderShape, double, Length, getLength, setLength);
}

int CylinderShape::getType() const
{
	return SHAPE_TYPE_CYLINDER;
}

double CylinderShape::getLength() const
{
	return m_length;
}

double CylinderShape::getRadius() const
{
	return m_radius;
}


void CylinderShape::setLength(double length)
{
	m_length = length;
	updateAabb();
}

void CylinderShape::setRadius(double radius)
{
	m_radius = radius;
	updateAabb();
}

void CylinderShape::updateAabb()
{
	m_aabb.setEmpty();
	m_aabb.extend(Vector3d(-m_radius, -m_length / 2.0, -m_radius));
	m_aabb.extend(Vector3d(m_radius, m_length / 2.0, m_radius));
}

double CylinderShape::getVolume() const
{
	return M_PI * m_radius * m_radius * m_length;
}

SurgSim::Math::Matrix33d CylinderShape::getSecondMomentOfVolume() const
{
	const double volume = getVolume();
	const double coef    = 1.0 / 12.0 * volume;
	const double coefDir = 1.0 /  2.0 * volume;
	const double squareL = m_length * m_length;
	const double squareRadius = m_radius * m_radius;

	Matrix33d secondMoment;
	secondMoment.setZero();
	secondMoment.diagonal().setConstant(coef * (3.0 * squareRadius + squareL));
	secondMoment(1, 1) = coefDir * (squareRadius);

	return secondMoment;
}

bool CylinderShape::isValid() const
{
	return (m_length >= 0) && (m_radius >= 0);
}

std::shared_ptr<Shape> CylinderShape::getCopy() const
{
	return std::make_shared<CylinderShape>(*this);
}

}; // namespace Math
}; // namespace SurgSim
