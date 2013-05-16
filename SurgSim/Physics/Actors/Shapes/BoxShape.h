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

#ifndef SURGSIM_PHYSICS_BOXSHAPE_H
#define SURGSIM_PHYSICS_BOXSHAPE_H

#include <SurgSim/Physics/Actors/Shapes/RigidShape.h>

namespace SurgSim 
{

namespace Physics
{

/// Box shape: box centered on (0 0 0), aligned with the axis
/// with different sizes along X, Y and Z
class BoxShape: public RigidShape
{
public:
	/// Constructor
	/// \param sizeX, sizeY, sizeZ the box sizes in all 3 directions (in m)
	BoxShape(double sizeX, double sizeY, double sizeZ)
	{
		m_size[0] = sizeX;
		m_size[1] = sizeY;
		m_size[2] = sizeZ;
	}

	/// Get size in X direction
	/// \return the size in the X direction (in m)
	double getSizeX() const
	{
		return m_size[0];
	}

	/// Get size in Y direction
	/// \return the size in the Y direction (in m)
	double getSizeY() const
	{
		return m_size[1];
	}

	/// Get size in Z direction
	/// \return the size in the Z direction (in m)
	double getSizeZ() const
	{
		return m_size[2];
	}

	/// Calculate the volume of the box
	/// \return The volume of the box (in m-3)
	double calculateVolume() const
	{
		return m_size[0] * m_size[1] * m_size[2];
	}

	/// Calculate the mass center of the box
	/// \return The mass center of the box
	Vector3d calculateMassCenter() const
	{
		return Vector3d(0.0, 0.0, 0.0);
	}

	/// Calculate the inertia of the box
	/// \param rho The mass density (in Kg.m-3)
	/// \return The 3x3 symmetric inertia matrix of the box
	Matrix33d calculateInertia(double rho) const
	{
		const double mass = calculateMass(rho);

		const double SquarelengthX = m_size[0] * m_size[0];
		const double SquarelengthY = m_size[1] * m_size[1];
		const double SquarelengthZ = m_size[2] * m_size[2];
		const double coef = 1.0 / 12.0 * mass;

		Matrix33d inertia;
		inertia.setZero();
		inertia(0, 0) = coef * (SquarelengthY + SquarelengthZ);
		inertia(1, 1) = coef * (SquarelengthX + SquarelengthZ);
		inertia(2, 2) = coef * (SquarelengthX + SquarelengthY);

		return inertia;
	}

private:
	/// The box sizes along the 3 axis respectively {X,Y,Z}
	double   m_size[3];
};

}; /// Physics

}; /// SurgSim

#endif /// SURGSIM_PHYSICS_BOXSHAPE_H
