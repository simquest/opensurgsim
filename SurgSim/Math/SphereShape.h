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

#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{

namespace Math
{
SURGSIM_STATIC_REGISTRATION(SphereShape);

/// Sphere shape: sphere centered on (0 0 0), defined with radius
class SphereShape: public Shape
{
public:
	/// Constructor
	/// \param radius The sphere radius (in m)
	explicit SphereShape(double radius = 0.0);

	SURGSIM_CLASSNAME(SurgSim::Math::SphereShape);

	/// \return the type of the shape
	virtual int getType() const override;

	/// Get the sphere radius
	/// \return The sphere radius
	double getRadius() const;

	/// Get the volume of the shape
	/// \return The volume of the shape (in m-3)
	virtual double getVolume() const override;

	/// Get the volumetric center of the shape
	/// \return The center of the shape
	virtual Vector3d getCenter() const override;

	/// Get the second central moment of the volume, commonly used
	/// to calculate the moment of inertia matrix
	/// \return The 3x3 symmetric second moment matrix
	virtual Matrix33d getSecondMomentOfVolume() const override;

	/// \return True if radius is bigger than or equal to 0; Otherwise, false.
	virtual bool isValid() const override;

protected:
	// Setters in 'protected' sections are for serialization purpose only.

	/// Set the sphere radius
	/// \param radius	The sphere radius
	void setRadius(double radius);

private:
	/// Sphere radius
	double m_radius;
};

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_SPHERESHAPE_H
