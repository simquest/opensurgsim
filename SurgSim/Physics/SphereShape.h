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

#ifndef SURGSIM_PHYSICS_SPHERESHAPE_H
#define SURGSIM_PHYSICS_SPHERESHAPE_H

#include <SurgSim/Physics/RigidShape.h>

namespace SurgSim
{

namespace Physics
{

/// Sphere shape: sphere centered on (0 0 0), defined with radius
class SphereShape: public RigidShape
{
public:
	/// Constructor
	/// \param radius The sphere radius (in m)
	explicit SphereShape(double radius);

	/// \return the type of the shape
	virtual int getType();

	/// Get the sphere radius
	/// \return The sphere radius
	double getRadius() const;

	/// Calculate the volume of the sphere
	/// \return The volume of the sphere (in m-3)
	virtual double calculateVolume() const;

	/// Calculate the mass center of the sphere
	/// \return The mass center of the sphere
	virtual Vector3d calculateMassCenter() const;

	/// Calculate the inertia of the sphere
	/// \param rho The mass density (in Kg.m-3)
	/// \return The 3x3 symmetric inertia matrix of the sphere
	virtual Matrix33d calculateInertia(double rho) const;

private:
	/// Sphere radius
	double m_radius;
};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_SPHERESHAPE_H
