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

#include <SurgSim/Physics/Actors/Shapes/RigidShape.h>

namespace SurgSim 
{

namespace Physics
{

/// Cylinder shape: centered on (0 0 0) with inner and outer radius
/// Cannot have inner radius = outer radius
/// /tparam direction The direction of the cylinder {X, Y, Z}
template <ShapeDirection direction>
class CylinderShape: public RigidShape
{
public:
	/// Constructor
	/// \param length The full length of the cylinder (in m)
	/// \param outerRadius, innerRadius The outer and inner radii (in m)
	/// \note outerRadius MUST BE DIFFERENT THAN inerRadius
	CylinderShape(double length, double outerRadius, double innerRadius = 0.0)
	{
		m_length      = length;
		m_outerRadius = outerRadius;
		m_innerRadius = innerRadius;
		SURGSIM_ASSERT(outerRadius != innerRadius);
	}

	/// Get the cylinder length
	/// \return The cylinder length (in m)
	double getLength() const
	{
		return m_length;
	}

	/// Get the cylinder outer radius
	/// \return The cylinder outer radius (in m)
	double getOuterRadius() const
	{
		return m_outerRadius;
	}

	/// Get the cylinder inner radius
	/// \return The cylinder inner radius (in m)
	double getInnerRadius() const
	{
		return m_innerRadius;
	}

	/// Calculate the volume of the cylinder
	/// \return The volume of the cylinder (in m-3)
	double calculateVolume() const
	{
		const double squareORadius = m_outerRadius * m_outerRadius;
		const double squareIRadius = m_innerRadius * m_innerRadius;

		return M_PI * (squareORadius - squareIRadius) * m_length;
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
		const double squareORadius = m_outerRadius * m_outerRadius;
		const double squareIRadius = m_innerRadius * m_innerRadius;
		const double mass = calculateMass(rho);

		const double coef    = 1.0 / 12.0 * mass;
		const double coefDir = 1.0 /  2.0 * mass;
		const double squareL = m_length * m_length;
		const double squareRadii = squareORadius + squareIRadius;

		Matrix33d inertia;
		inertia.setZero();
		inertia.diagonal().setConstant(coef * (3.0 * squareRadii + squareL));
		inertia(direction, direction) = coefDir * (squareRadii);

		return inertia;
	}

private:
	/// The cylinder outer radius
	double   m_outerRadius;

	/// The cylinder inner radius
	double   m_innerRadius;
	
	/// The cylinder length
	double   m_length;
};

}; /// Physics

}; /// SurgSim

#endif /// SURGSIM_PHYSICS_CYLINDERSHAPE_H
