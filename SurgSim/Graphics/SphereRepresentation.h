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

#ifndef SURGSIM_GRAPHICS_SPHEREREPRESENTATION_H
#define SURGSIM_GRAPHICS_SPHEREREPRESENTATION_H

#include "SurgSim/Graphics/Representation.h"

namespace SurgSim
{

namespace Graphics
{

/// Base graphics sphere representation class, which defines the basic interface for a sphere that can be visualized.
/// The sphere center is at (0, 0, 0).
class SphereRepresentation : public virtual Representation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	/// \post	The sphere radius is 1.0.
	explicit SphereRepresentation(const std::string& name) : Representation(name)
	{
	}

	/// Sets the radius of the sphere
	virtual void setRadius(double radius) = 0;
	/// Returns the radius of the sphere
	virtual double getRadius() const = 0;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_SPHEREREPRESENTATION_H
