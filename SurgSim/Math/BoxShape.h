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

#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{

namespace Math
{
SURGSIM_STATIC_REGISTRATION(BoxShape);

/// Box shape: box centered on (0 0 0), aligned with the axis
/// with different sizes along X, Y and Z
class BoxShape: public Shape
{
public:
	/// Constructor
	/// \param sizeX, sizeY, sizeZ the box sizes in all 3 directions (in m)
	explicit BoxShape(double sizeX = 0.0, double sizeY = 0.0, double sizeZ = 0.0);

	explicit BoxShape(const BoxShape& other);

	SURGSIM_CLASSNAME(SurgSim::Math::BoxShape);

	/// \return the type of the shape
	int getType() const override;

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

	/// Get the volume of the shape
	/// \return The volume of the shape (in m-3)
	double getVolume() const override;

	/// Get the second central moment of the volume, commonly used
	/// to calculate the moment of inertia matrix
	/// \return The 3x3 symmetric second moment matrix
	Matrix33d getSecondMomentOfVolume() const override;

	/// Function that returns the local vertex location, given an index.
	/// \param i The vertex index.
	/// \return The local vertex position.
	Vector3d getVertex(const int i) const;

	/// Function that returns the local vertices' location
	/// \return All eight vertices of the box
	const std::array<Vector3d, 8>& getVertices() const;

	/// \return True if size along X, Y, Z are bigger than or equal to 0; Otherwise, false.
	bool isValid() const override;

	std::shared_ptr<Shape> getCopy() const override;

protected:
	// Setters in 'protected' sections are for serialization purpose only.

	/// Set size in X direction
	/// \param sizeX	the size in the X direction (in m)
	void setSizeX(double sizeX);

	/// Set size in Y direction
	/// \param sizeY	the size in the Y direction (in m)
	void setSizeY(double sizeY);

	/// Set size in Z direction
	/// \param sizeZ	the size in the Z direction (in m)
	void setSizeZ(double sizeZ);

private:
	/// Function that calculates the box vertices.
	void calculateVertices();

	void updateAabb() const override;

	/// The box sizes along the 3 axis respectively {X,Y,Z}
	Vector3d m_size;

	/// The box vertices.
	std::array<Vector3d, 8> m_vertices;
};

}; // Math

}; // SurgSim

#endif // SURGSIM_MATH_BOXSHAPE_H
