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

#include <SurgSim/Physics/CapsuleShape.h>

namespace SurgSim
{
namespace Physics
{

CapsuleShape::CapsuleShape(double length, double radius)
{
	m_length = length;
	m_radius = radius;
}

int CapsuleShape::getType()
{
	return RIGID_SHAPE_TYPE_CAPSULE;
}

double CapsuleShape::getLength() const
{
	return m_length;
}

double CapsuleShape::getRadius() const
{
	return m_radius;
}

double CapsuleShape::calculateVolume() const
{
	const double r2 = m_radius * m_radius;
	const double localCylinderVolume = M_PI * (r2) * m_length;
	const double localSphereVolume   = 4.0 / 3.0 * M_PI * r2 * m_radius;

	return localCylinderVolume + localSphereVolume;
}

SurgSim::Math::Vector3d CapsuleShape::calculateMassCenter() const
{
	return Vector3d(0.0, 0.0, 0.0);
}

SurgSim::Math::Vector3d CapsuleShape::topCentre() const
{
	return Vector3d(0.0, m_length / 2.0, 0.0);
}

SurgSim::Math::Vector3d CapsuleShape::bottomCentre() const
{
	return Vector3d(0.0, -m_length / 2.0, 0.0);
}

SurgSim::Math::Matrix33d CapsuleShape::calculateInertia(double rho) const
{
	const double &r = m_radius;
	const double &l = m_length;
	const double r2 = r * r;
	const double l2 = l * l;
	const double cylinderMass = rho * M_PI * (r2) * l;
	const double sphereMass   = rho * 4.0 / 3.0 * M_PI * r2 * r;

	// The Inertia matrix is a combination of the cylinder inertia and
	// the 2 hemispheres inertia:
	//
	// Inertia of cylinder along the Y axis
	// mc = PI.radius.radius.length (mass of the cylinder)
	// a = 1/2.mc.r^2
	// b = 1/12.mc.(3.r^2 + h^2)
	//               (b 0 0)
	// I(cylinder) = (0 a 0)
	//               (0 0 b)
	//
	// Inertia of the 2 hemispheres along the X axis (direction = 0)
	// ms = 4/3 pi.radius.radius.radius (mass of the entire sphere)
	// c = 2/5.ms.r^2
	// d = 2/5.ms.r^2 + ms.h^2/4 + 3/8.ms.r.h
	//                    (d 0 0)
	// I(2 hemispheres) = (0 c 0)
	//                    (0 0 d)
	double a = 1.0 / 2.0  * cylinderMass * r2;
	double b = 1.0 / 12.0 * cylinderMass * (3.0 * r2 + l2);
	double c = 2.0 / 5.0  * sphereMass   * r2;
	double d = c + sphereMass * l * (l / 4.0 + 3.0 / 8.0 * r);

	Matrix33d inertia;
	inertia.setZero();
	inertia.diagonal().setConstant(b + d);
	inertia(1, 1) = a + c;

	return inertia;
}

}; // namespace Physics
}; // namespace SurgSim
