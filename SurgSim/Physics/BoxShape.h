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

#include <array>

#include <SurgSim/Physics/RigidShape.h>
#include <SurgSim/Math/Quaternion.h>

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
	BoxShape(double sizeX, double sizeY, double sizeZ) :
		m_size(Vector3d(sizeX, sizeY, sizeZ))
	{
		calculateVertices();
	}


	/// \return the type of the shape
	int getType()
	{
		return RIGID_SHAPE_TYPE_BOX;
	}

	Vector3d getSize() const
	{
		return m_size;
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

		const Vector3d sizeSquared = m_size.array() * m_size.array();
		const double coef = 1.0 / 12.0 * mass;
		Matrix33d inertia = Matrix33d::Zero();
		inertia.diagonal() = coef * Vector3d(sizeSquared[1] + sizeSquared[2],
											sizeSquared[0] + sizeSquared[2],
											sizeSquared[0] + sizeSquared[1]);
		return inertia;
	}

	/// Function that returns the local vertex location, given an index.
	/// \param i The vertex index.
	/// \return The local vertex position.
	Vector3d getVertex(const int i) const
	{
		return m_vertices[i];
	}

	const std::array<Vector3d, 8>& getVertices() const
	{
		return m_vertices;
	}

private:
	/// Function that calculates the box vertices.
	void calculateVertices()
	{
		static const std::array<Vector3d, 8> multiplier = {Vector3d(-0.5, -0.5, -0.5),
														   Vector3d(-0.5, -0.5,  0.5),
														   Vector3d(-0.5,  0.5,  0.5),
														   Vector3d(-0.5,  0.5, -0.5),
														   Vector3d( 0.5, -0.5, -0.5),
														   Vector3d( 0.5, -0.5,  0.5),
														   Vector3d( 0.5,  0.5,  0.5),
														   Vector3d( 0.5,  0.5, -0.5)};
		for(int i = 0; i < 8; ++i)
		{
			m_vertices[i] = m_size.array() * multiplier[i].array();
		}
	}

private:
	/// The box sizes along the 3 axis respectively {X,Y,Z}
	Vector3d m_size;

	/// The box vertices.
	std::array<Vector3d,8> m_vertices;
};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_BOXSHAPE_H
