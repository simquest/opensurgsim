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

#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{

namespace Math
{
SURGSIM_STATIC_REGISTRATION(CapsuleShape);

/// Capsule shape: centered on (0, 0, 0), aligned along Y,
/// with length and radius
class CapsuleShape: public Shape
{
public:
	/// Constructor
	/// \param length The capsule length (i.e. of the cylinder) (in m)
	/// \param radius The capsule radius (i.e. of the cylinder/spheres) (in m)
	CapsuleShape(double length = 0.0, double radius = 0.0);

	SURGSIM_CLASSNAME(SurgSim::Math::CapsuleShape);

	/// \return the type of the shape
	virtual int getType() const override;

	/// Get the capsule length (i.e. cylinder length)
	/// \return The capsule length (in m)
	double getLength() const;

	/// Get the capsule radius (i.e. cylinder/spheres radius)
	/// \return The capsule radius (in m)
	double getRadius() const;

	/// Get the volume of the shape
	/// \return The volume of the shape (in m-3)
	virtual double getVolume() const override;

	/// Get the volumetric center of the shape
	/// \return The center of the shape
	virtual Vector3d getCenter() const override;

	/// Return the center of the top sphere of the internal cylinder
	/// \return The top center of the sphere of the capsule
	Vector3d topCenter() const;

	/// Return the center of the bottom sphere of the internal cylinder
	/// \return The bottom center of the sphere of the capsule
	Vector3d bottomCenter() const;

	/// Get the second central moment of the volume, commonly used
	/// to calculate the moment of inertia matrix
	/// \return The 3x3 symmetric second moment matrix
	virtual Matrix33d getSecondMomentOfVolume() const override;

	/// \return True if length and radius are bigger than or equal to 0; Otherwise, false.
	virtual bool isValid() const override;

protected:
	// Setters in 'protected' sections are for serialization purpose only.

	/// Set the capsule length (i.e. cylinder length)
	/// \param length	The capsule length (in m)
	void setLength(double length);

	/// Set the capsule radius (i.e. cylinder/spheres radius)
	/// \param radius	The capsule radius (in m)
	void setRadius(double radius);

private:
	/// Capsule length
	double m_length;

	/// Capsule radius
	double m_radius;
};

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_CAPSULESHAPE_H
