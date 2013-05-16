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

#ifndef SURGSIM_PHYSICS_SPHERESHAPE_H
#define SURGSIM_PHYSICS_SPHERESHAPE_H

#include <SurgSim/Physics/Actors/Shapes/RigidShape.h>

namespace SurgSim 
{

namespace Physics
{

/// Sphere shape: sphere centered on (0 0 0), with radius, solid or hollow
class SphereShape: public RigidShape
{
public:
	/// Constructor
	/// \param radius The sphere radius (in m)
	/// \param isSolid True if the sphere is solid, False if it is hollow
	SphereShape(double radius, bool isSolid = true)
	{
		m_radius = radius;
		m_isSolid  = isSolid;
	}

	/// Get the sphere radius
	/// \return The sphere radius
	double getRadius() const
	{
		return m_radius;
	}

	/// Query if the sphere is solid or not
	/// \return True if the sphere is solid, False if it is hollow
	bool isSolid() const
	{
		return m_isSolid;
	}

	/// Calculate the volumeof the sphere
	/// \return The volume of the sphere (in m-3)
	double calculateVolume() const
	{
		double r2 = m_radius * m_radius;

		if (m_isSolid)
		{
			return 4.0 / 3.0 * M_PI * m_radius * r2;
		}
		else
		{
			return 4.0 * M_PI * r2;
		}
	}

	/// Calculate the mass center of the sphere
	/// \return The mass center of the sphere
	Vector3d calculateMassCenter() const
	{
		return Vector3d(0.0, 0.0, 0.0);
	}

	/// Calculate the inertia of the sphere
	/// \param rho The mass density (in Kg.m-3)
	/// \return The 3x3 symmetric inertia matrix of the sphere
	Matrix33d calculateInertia(double rho) const
	{
		const double mass = calculateMass(rho);

		double diagonalCoefficient = (m_isSolid ? 2.0 / 5.0 : 2.0 / 3.0);
		diagonalCoefficient *= mass * m_radius * m_radius;

		Matrix33d inertia;
		inertia.setZero();
		inertia.diagonal().setConstant(diagonalCoefficient);

		return inertia;
	}

private:
	/// Sphere radius
	double m_radius;

	/// Is the sphere solid or hollow ?
	bool m_isSolid;
};

}; /// Physics

}; /// SurgSim

#endif /// SURGSIM_PHYSICS_SPHERESHAPE_H
