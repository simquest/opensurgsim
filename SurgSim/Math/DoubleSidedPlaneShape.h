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

#ifndef SURGSIM_MATH_DOUBLESIDEDPLANESHAPE_H
#define SURGSIM_MATH_DOUBLESIDEDPLANESHAPE_H

#include <SurgSim/Math/Shape.h>

namespace SurgSim
{

namespace Math
{

/// DoubleSidedPlaneShape: The XZ plane (d = 0) with normal pointing along
/// positive Y axis.
class DoubleSidedPlaneShape: public Shape
{
public:

	/// Constructor: No members to initialize.
	DoubleSidedPlaneShape();

	/// \return the type of the shape
	virtual int getType();

	/// Calculate the volume of the plane
	/// \return The volume of the plane, which is 0
	virtual double calculateVolume() const;

	/// Calculate the mass center of the plane
	/// \return The mass center of the plane
	virtual Vector3d calculateMassCenter() const;

	/// Calculate the inertia of the box
	/// \param rho The mass density (in Kg.m-3)
	/// \return The 3x3 symmetric inertia matrix of the plane
	virtual Matrix33d calculateInertia(double rho) const;

	/// Gets the d of the plane equation.
	/// \return	The value of d (always 0).
	double getD() const;

	/// Gets the normal of the plane equation.
	/// \return	The value of the normal (always Y axis).
	Vector3d getNormal() const;

};

}; // Math

}; // SurgSim

#endif // SURGSIM_MATH_DOUBLESIDEDPLANESHAPE_H
