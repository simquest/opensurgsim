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

#ifndef SURGSIM_PHYSICS_CYLINDERSHAPE_H
#define SURGSIM_PHYSICS_CYLINDERSHAPE_H

#include <SurgSim/Framework/Assert.h>

#include <SurgSim/Physics/Actors/RigidShape.h>

namespace SurgSim
{

namespace Physics
{

/// Cylinder shape: centered on (0 0 0) defined with length and radius
/// \tparam direction The direction of the cylinder {X, Y, Z}
template <ShapeDirection direction>
class CylinderShape: public RigidShape
{
public:
	/// Constructor
	/// \param length The length of the cylinder (in m)
	/// \param radius The cylinder radius (in m)
	CylinderShape(double length, double radius)
	{
		m_length = length;
		m_radius = radius;
	}

	/// \return the type of the shape
	int getType()
	{
		return RIGID_SHAPE_TYPE_CYLINDER;
	}

	/// Get the cylinder length
	/// \return The cylinder length (in m)
	double getLength() const
	{
		return m_length;
	}

	/// Get the cylinder radius
	/// \return The cylinder radius (in m)
	double getRadius() const
	{
		return m_radius;
	}

	/// Calculate the volume of the cylinder
	/// \return The volume of the cylinder (in m-3)
	double calculateVolume() const
	{
		return M_PI * m_radius * m_radius * m_length;
	}

	/// Calculate the mass center of the cylinder
	/// \return The mass center of the cylinder
	Vector3d calculateMassCenter() const
	{
		return Vector3d(0.0, 0.0, 0.0);
	}

	/// Calculate the inertia of the cylinder
	/// \param rho The mass density (in Kg.m-3)
	/// \return The 3x3 symmetric inertia matrix of the cylinder
	Matrix33d calculateInertia(double rho) const
	{
		const double mass = calculateMass(rho);
		const double coef    = 1.0 / 12.0 * mass;
		const double coefDir = 1.0 /  2.0 * mass;
		const double squareL = m_length * m_length;
		const double squareRadius = m_radius * m_radius;

		Matrix33d inertia;
		inertia.setZero();
		inertia.diagonal().setConstant(coef * (3.0 * squareRadius + squareL));
		inertia(direction, direction) = coefDir * (squareRadius);

		return inertia;
	}

private:
	/// The cylinder radius
	double   m_radius;

	/// The cylinder length
	double   m_length;
};

}; /// Physics

}; /// SurgSim

#endif /// SURGSIM_PHYSICS_CYLINDERSHAPE_H
