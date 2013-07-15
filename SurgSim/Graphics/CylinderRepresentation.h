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

#ifndef SURGSIM_GRAPHICS_CYLINDERREPRESENTATION_H
#define SURGSIM_GRAPHICS_CYLINDERREPRESENTATION_H

#include <SurgSim/Graphics/Representation.h>

namespace SurgSim
{

namespace Graphics
{

/// Base graphics cylinder representation class,
/// which defines the basic interface for a cylinder that can be visualized.
/// The cylinder center is at (0, 0, 0).
class CylinderRepresentation : public virtual Representation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	/// \post	The cylinder radius is 1.0.
	explicit CylinderRepresentation(const std::string& name) : Representation(name)
	{
	}

	/// Sets the radius of the cylinder
	virtual void setRadius(double radius) = 0;
	/// Returns the radius of the cylinder
	virtual double getRadius() const = 0;

	/// Sets the height of the cylinder
	virtual void setHeight(double height) = 0;
	/// Returns the height of the cylinder
	virtual double getHeight() const = 0;

	/// Sets the size of the cylinder
	/// \param radius Size along X-axis of the cylinder
	/// \param height Size along Y-axis of the cylinder
	virtual void setSize(double radius, double height) = 0;
	/// Gets the size of the cylinder
	/// \param radius Reference to store the size along X-axis of the cylinder
	/// \param height Reference to store the size along Y-axis of the cylinder
	virtual void getSize(double* radius, double* height) = 0;

	/// Sets the size of the cylinder
	/// \param size Size of the cylinder
	virtual void setSize(SurgSim::Math::Vector2d size) = 0;
	/// Returns the radius of the cylinder
	/// \return Size of the cylinder
	virtual SurgSim::Math::Vector2d getSize() const = 0;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_CYLINDERREPRESENTATION_H
