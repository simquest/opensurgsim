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

#ifndef SURGSIM_PHYSICS_PLANESHAPE_H
#define SURGSIM_PHYSICS_PLANESHAPE_H

#include <SurgSim/Physics/RigidShape.h>

namespace SurgSim
{

namespace Physics
{

/// PlaneShape: The XZ plane (d = 0) with normal pointing along
/// positive Y axis.
class PlaneShape: public RigidShape
{
public:

	/// Constructor: No members to initialize.
	PlaneShape()
	{

	}

	/// \return the type of the shape
	int getType()
	{
		return RIGID_SHAPE_TYPE_PLANE;
	}

	/// Calculate the volume of the plane
	/// \return The volume of the plane, which is 0
	double calculateVolume() const
	{
		return 0;
	}

	/// Calculate the mass center of the plane
	/// \return The mass center of the plane
	Vector3d calculateMassCenter() const
	{
		return Vector3d(0.0, 0.0, 0.0);
	}

	/// Calculate the inertia of the box
	/// \param rho The mass density (in Kg.m-3)
	/// \return The 3x3 symmetric inertia matrix of the plane
	Matrix33d calculateInertia(double rho) const
	{
		return Matrix33d::Identity();
	}

	/// Gets the d of the plane equation.
	/// \return	The value of d (always 0).
	inline double getD()
	{
		return 0.0;
	}

	/// Gets the normal of the plane equation.
	/// \return	The value of the normal (always Y axis).
	inline Vector3d getNormal()
	{
		return Vector3d(0.0, 1.0, 0.0);
	}

};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_PLANESHAPE_H
