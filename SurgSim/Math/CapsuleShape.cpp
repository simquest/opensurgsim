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

#include "SurgSim/Math/CapsuleShape.h"

namespace SurgSim
{
namespace Math
{
SURGSIM_REGISTER(SurgSim::Math::Shape, SurgSim::Math::CapsuleShape, CapsuleShape);

CapsuleShape::CapsuleShape(double length, double radius) : m_length(length), m_radius(radius)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(CapsuleShape, double, Radius, getRadius, setRadius);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(CapsuleShape, double, Length, getLength, setLength);
	updateAabb();
}

CapsuleShape::CapsuleShape(const CapsuleShape& other) :
	Shape(other.getPose()),
	m_length(other.getLength()),
	m_radius(other.getRadius())
{
	updateAabb();
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(CapsuleShape, double, Radius, getRadius, setRadius);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(CapsuleShape, double, Length, getLength, setLength);
}

int CapsuleShape::getType() const
{
	return SHAPE_TYPE_CAPSULE;
}

double CapsuleShape::getLength() const
{
	return m_length;
}

double CapsuleShape::getRadius() const
{
	return m_radius;
}

void CapsuleShape::setLength(double length)
{
	m_length = length;
	updateAabb();
}

void CapsuleShape::setRadius(double radius)
{
	m_radius = radius;
	updateAabb();
}

double CapsuleShape::getVolume() const
{
	const double r2 = m_radius * m_radius;
	const double localCylinderVolume = M_PI * (r2) * m_length;
	const double localSphereVolume   = 4.0 / 3.0 * M_PI * r2 * m_radius;

	return localCylinderVolume + localSphereVolume;
}

SurgSim::Math::Vector3d CapsuleShape::topCenter() const
{
	return Vector3d(0.0, m_length / 2.0, 0.0);
}

SurgSim::Math::Vector3d CapsuleShape::bottomCenter() const
{
	return Vector3d(0.0, -m_length / 2.0, 0.0);
}

SurgSim::Math::Matrix33d CapsuleShape::getSecondMomentOfVolume() const
{
	const double& r = m_radius;
	const double& l = m_length;
	const double r2 = r * r;
	const double l2 = l * l;
	const double cylinderVolume = M_PI * (r2) * l;
	const double sphereVolume = 4.0 / 3.0 * M_PI * r2 * r;

	// The matrix is a combination of the cylinder and
	// the 2 hemispheres:
	//
	// Second central moment of cylinder along the Y axis
	// vc = PI.radius.radius.length (volume of the cylinder)
	// a = 1/2.vc.r^2
	// b = 1/12.vc.(3.r^2 + h^2)
	//               (b 0 0)
	// I(cylinder) = (0 a 0)
	//               (0 0 b)
	//
	// Second central moment of the 2 hemispheres along the X axis (direction = 0)
	// vs = 4/3 pi.radius.radius.radius (volume of the entire sphere)
	// c = 2/5.vs.r^2
	// d = 2/5.vs.r^2 + vs.h^2/4 + 3/8.vs.r.h
	//                    (d 0 0)
	// I(2 hemispheres) = (0 c 0)
	//                    (0 0 d)
	double a = 1.0 / 2.0  * cylinderVolume * r2;
	double b = 1.0 / 12.0 * cylinderVolume * (3.0 * r2 + l2);
	double c = 2.0 / 5.0  * sphereVolume * r2;
	double d = c + sphereVolume * l * (l / 4.0 + 3.0 / 8.0 * r);

	Matrix33d secondMoment;
	secondMoment.setZero();
	secondMoment.diagonal().setConstant(b + d);
	secondMoment(1, 1) = a + c;

	return secondMoment;
}

bool CapsuleShape::isValid() const
{
	return (m_length >= 0) && (m_radius >= 0);
}

void CapsuleShape::updateAabb() const
{
	Aabbd aabb;
	aabb.extend(Vector3d(-m_radius, -m_length / 2.0 - m_radius, -m_radius));
	aabb.extend(Vector3d(m_radius, m_length / 2.0 + m_radius, m_radius));
	m_aabb = transformAabb(m_pose, aabb);
}

std::shared_ptr<Shape> CapsuleShape::getCopy() const
{
	return std::make_shared<CapsuleShape>(*this);
}

}; // namespace Math
}; // namespace SurgSim
