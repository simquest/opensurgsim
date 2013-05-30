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

#ifndef SURGSIM_PHYSICS_CAPSULESHAPE_H
#define SURGSIM_PHYSICS_CAPSULESHAPE_H

#include <SurgSim/Physics/RigidShape.h>

namespace SurgSim
{

namespace Physics
{

/// Capsule shape: centered on (0 0 0), with length and radius
/// \tparam direction The shape direction in {X, Y, Z}
template <ShapeDirection direction>
class CapsuleShape: public RigidShape
{
public:
	/// Constructor
	/// \param length The capsule length (i.e. of the cylinder) (in m)
	/// \param radius The capsule radius (i.e. of the cylinder/spheres) (in m)
	CapsuleShape(double length, double radius)
	{
		m_length = length;
		m_radius = radius;
	}

	/// \return the type of the shape
	int getType()
	{
		return RIGID_SHAPE_TYPE_CAPSULE;
	}

	/// Get the capsule length (i.e. cylinder length)
	/// \return The capsule length (in m)
	double getLength() const
	{
		return m_length;
	}

	/// Get the capsule radius (i.e. cylinder/spheres radius)
	/// \return The capsule radius (in m)
	double getRadius() const
	{
		return m_radius;
	}

	/// Calculate the volume of the capsule
	/// \return The volume of the capsule (in m-3)
	double calculateVolume() const
	{
		const double r2 = m_radius * m_radius;
		const double localCylinderVolume = M_PI * (r2) * m_length;
		const double localSphereVolume   = 4.0 / 3.0 * M_PI * r2 * m_radius;

		return localCylinderVolume + localSphereVolume;
	}

	/// Calculate the mass center of the capsule
	/// \return The mass center of the capsule
	Vector3d calculateMassCenter() const
	{
		return Vector3d(0.0, 0.0, 0.0);
	}

	/// Calculate the inertia from the capsule
	/// \param rho The mass density (in Kg.m-3)
	/// \return The 3x3 symmetric inertia matrix of the capsule
	Matrix33d calculateInertia(double rho) const
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
		// Inertia of cylinder along the X axis (direction = 0)
		// mc = PI.radius.radius.length (mass of the cylinder)
		// a = 1/2.mc.r^2
		// b = 1/12.mc.(3.r^2 + h^2)
		//               (a 0 0)
		// I(cylinder) = (0 b 0)
		//               (0 0 b)
		//
		// Inertia of the 2 hemispheres along the X axis (direction = 0)
		// ms = 4/3 pi.radius.radius.radius (mass of the entire sphere)
		// c = 2/5.ms.r^2
		// d = 2/5.ms.r^2 + ms.h^2/4 + 3/8.ms.r.h
		//                    (c 0 0)
		// I(2 hemispheres) = (0 d 0)
		//                    (0 0 d)
		double a = 1.0 / 2.0  * cylinderMass * r2;
		double b = 1.0 / 12.0 * cylinderMass * (3.0 * r2 + l2);
		double c = 2.0 / 5.0  * sphereMass   * r2;
		double d = c + sphereMass * l * (l / 4.0 + 3.0 / 8.0 * r);

		Matrix33d inertia;
		inertia.setZero();
		inertia.diagonal().setConstant(b + d);
		inertia(direction, direction) = a + c;

		return inertia;
	}

private:
	/// Capsule radius
	double   m_radius;

	/// Capsule length
	double   m_length;
};

}; /// Physics

}; /// SurgSim

#endif /// SURGSIM_PHYSICS_CAPSULESHAPE_H
