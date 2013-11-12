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

#ifndef SURGSIM_MATH_PLANESHAPE_H
#define SURGSIM_MATH_PLANESHAPE_H

#include <SurgSim/Math/Shape.h>

namespace SurgSim
{

namespace Math
{

/// The XZ plane (d = 0) with normal pointing along positive Y axis.
/// The difference between PlaneShape and DoubleSidedPlane shape is in the way the Physics scene
/// handles these in terms of collision detection. While the DoubleSidedPlane is handled as a thin
/// solid of (essentially) zero thickness and space on either side it, the PlaneShape is considered
/// to be a entirely solid on the side of the plane which is opposite to the normal. It is made of
/// space only on the positive side of the plane normal. This results in having a robust collision
/// object which does not let any objects through, and could be useful to define the scene floor.
class PlaneShape: public Shape
{
public:

	/// Constructor: No members to initialize.
	PlaneShape();

	/// \return the type of the shape
	virtual int getType() override;

	/// Calculate the volume of the plane
	/// \return The volume of the plane, which is 0
	virtual double calculateVolume() const override;

	/// Calculate the mass center of the plane
	/// \return The mass center of the plane
	virtual Vector3d calculateMassCenter() const override;

	/// Calculate the inertia of the box
	/// \param rho The mass density (in Kg.m-3)
	/// \return The 3x3 symmetric inertia matrix of the plane
	virtual Matrix33d calculateInertia(double rho) const override;

	/// Gets the d of the plane equation.
	/// \return	The value of d (always 0).
	double getD() const;

	/// Gets the normal of the plane equation.
	/// \return	The value of the normal (always Y axis).
	Vector3d getNormal() const;

};

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_PLANESHAPE_H
