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

#ifndef SURGSIM_MATH_SPHERESHAPE_H
#define SURGSIM_MATH_SPHERESHAPE_H

#include <SurgSim/Math/Shape.h>

namespace SurgSim
{

namespace Math
{

/// Sphere shape: sphere centered on (0 0 0), defined with radius
class SphereShape: public Shape
{
public:
	/// Constructor
	/// \param radius The sphere radius (in m)
	explicit SphereShape(double radius = 0);

	/// \return the type of the shape
	virtual int getType() override;

	/// Get the sphere radius
	/// \return The sphere radius
	double getRadius() const;

	/// Calculate the volume of the sphere
	/// \return The volume of the sphere (in m-3)
	virtual double calculateVolume() const override;

	/// Calculate the mass center of the sphere
	/// \return The mass center of the sphere
	virtual Vector3d calculateMassCenter() const override;

	/// Calculate the inertia of the sphere
	/// \param rho The mass density (in Kg.m-3)
	/// \return The 3x3 symmetric inertia matrix of the sphere
	virtual Matrix33d calculateInertia(double rho) const override;

	/// Serialize declarations of the sphere
	OSS_SERIALIZE(SurgSim::Math::SphereShape);

private:
	/// Sphere radius
	double m_radius;
};

}; // Math

}; // SurgSim

#endif // SURGSIM_MATH_SPHERESHAPE_H
