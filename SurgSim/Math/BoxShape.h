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

#ifndef SURGSIM_MATH_BOXSHAPE_H
#define SURGSIM_MATH_BOXSHAPE_H

#include <array>

#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Shape.h"

#include "SurgSim/Serialize/Convert.h"

namespace SurgSim
{

namespace Math
{

/// Box shape: box centered on (0 0 0), aligned with the axis
/// with different sizes along X, Y and Z
class BoxShape: public Shape
{
public:
	/// Constructor
	/// \param sizeX, sizeY, sizeZ the box sizes in all 3 directions (in m)
	BoxShape(double sizeX = 0.0, double sizeY = 0.0, double sizeZ = 0.0);

	/// \return the type of the shape
	virtual int getType() override;

	/// Get size of the box
	/// \return the size of the box (in m)
	Vector3d getSize() const;

	/// Get size in X direction
	/// \return the size in the X direction (in m)
	double getSizeX() const;

	/// Get size in Y direction
	/// \return the size in the Y direction (in m)
	double getSizeY() const;

	/// Get size in Z direction
	/// \return the size in the Z direction (in m)
	double getSizeZ() const;

	/// Calculate the volume of the box
	/// \return The volume of the box (in m-3)
	virtual double calculateVolume() const override;

	/// Calculate the mass center of the box
	/// \return The mass center of the box
	virtual Vector3d calculateMassCenter() const override;

	/// Calculate the inertia of the box
	/// \param rho The mass density (in Kg.m-3)
	/// \return The 3x3 symmetric inertia matrix of the box
	virtual Matrix33d calculateInertia(double rho) const override;

	/// Function that returns the local vertex location, given an index.
	/// \param i The vertex index.
	/// \return The local vertex position.
	Vector3d getVertex(const int i) const;

	/// Function that returns the local vertices' location
	/// \return All eight vertices of the box
	const std::array<Vector3d, 8>& getVertices() const;

	/// Serialize declarations of the box
	OSS_SERIALIZE(SurgSim::Math::BoxShape);

private:
	/// Function that calculates the box vertices.
	void calculateVertices();

	/// The box sizes along the 3 axis respectively {X,Y,Z}
	Vector3d m_size;

	/// The box vertices.
	std::array<Vector3d,8> m_vertices;
};

}; // Math

}; // SurgSim

#endif // SURGSIM_MATH_BOXSHAPE_H
