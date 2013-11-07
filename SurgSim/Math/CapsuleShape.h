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

#ifndef SURGSIM_MATH_CAPSULESHAPE_H
#define SURGSIM_MATH_CAPSULESHAPE_H

#include <SurgSim/Math/Shape.h>

namespace SurgSim
{

namespace Math
{

/// Capsule shape: centered on (0, 0, 0), aligned along Y,
/// with length and radius
class CapsuleShape: public Shape
{
public:
	/// Constructor
	/// \param length The capsule length (i.e. of the cylinder) (in m)
	/// \param radius The capsule radius (i.e. of the cylinder/spheres) (in m)
	CapsuleShape(double length = 0, double radius = 0);

	/// \return the type of the shape
	virtual int getType() override;

	/// Get the capsule length (i.e. cylinder length)
	/// \return The capsule length (in m)
	double getLength() const;

	/// Get the capsule radius (i.e. cylinder/spheres radius)
	/// \return The capsule radius (in m)
	double getRadius() const;

	/// Calculate the volume of the capsule
	/// \return The volume of the capsule (in m-3)
	virtual double calculateVolume() const override;

	/// Calculate the mass center of the capsule
	/// \return The mass center of the capsule
	virtual Vector3d calculateMassCenter() const override;

	/// Return the centre of the top sphere of the internal cylinder
	/// \return The top centre of the sphere of the capsule
	Vector3d topCentre() const;

	/// Return the centre of the bottom sphere of the internal cylinder
	/// \return The bottom centre of the sphere of the capsule
	Vector3d bottomCentre() const;

	/// Calculate the inertia from the capsule
	/// \param rho The mass density (in Kg.m-3)
	/// \return The 3x3 symmetric inertia matrix of the capsule
	virtual Matrix33d calculateInertia(double rho) const override;

	/// Serialize declarations of the capsule
	OSS_SERIALIZE(SurgSim::Math::CapsuleShape);

private:
	/// Capsule radius
	double   m_radius;

	/// Capsule length
	double   m_length;
};

}; // Math

}; // SurgSim

#endif // SURGSIM_MATH_CAPSULESHAPE_H
