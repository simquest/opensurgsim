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

#ifndef SURGSIM_MATH_CYLINDERSHAPE_H
#define SURGSIM_MATH_CYLINDERSHAPE_H

#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{

namespace Math
{
SURGSIM_STATIC_REGISTRATION(CylinderShape);

/// Cylinder shape: centered on (0 0 0), aligned along Y,
/// defined with length and radius.
class CylinderShape: public Shape
{
public:
	/// Constructor
	/// \param length The length of the cylinder (in m)
	/// \param radius The cylinder radius (in m)
	explicit CylinderShape(double length = 0.0, double radius = 0.0);

	explicit CylinderShape(const CylinderShape& other);

	SURGSIM_CLASSNAME(SurgSim::Math::CylinderShape);

	/// \return the type of the shape
	int getType() const override;

	/// Get the cylinder length
	/// \return The cylinder length (in m)
	double getLength() const;

	/// Get the cylinder radius
	/// \return The cylinder radius (in m)
	double getRadius() const;

	/// Get the volume of the shape
	/// \return The volume of the shape (in m-3)
	double getVolume() const override;

	/// Get the second central moment of the volume, commonly used
	/// to calculate the moment of inertia matrix
	/// \return The 3x3 symmetric second moment matrix
	Matrix33d getSecondMomentOfVolume() const override;

	/// \return True if length and radius are bigger than or equal to 0; Otherwise, false.
	bool isValid() const override;

	std::shared_ptr<Shape> getCopy() const override;

protected:
	// Setters in 'protected' sections are for serialization purpose only.

	/// Set the cylinder length
	/// \param length	The capsule length (in m)
	void setLength(double length);

	/// Set the cylinder radius
	/// \param radius	The capsule radius (in m)
	void setRadius(double radius);

private:

	void updateAabb();

	/// The cylinder length
	double m_length;

	/// The cylinder radius
	double m_radius;
};

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_CYLINDERSHAPE_H
