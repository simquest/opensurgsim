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

#ifndef SURGSIM_GRAPHICS_CAPSULEREPRESENTATION_H
#define SURGSIM_GRAPHICS_CAPSULEREPRESENTATION_H

#include "SurgSim/Graphics/Representation.h"

namespace SurgSim
{

namespace Graphics
{

/// Base graphics capsule representation class, which defines the basic interface for a capsule that can be visualized.
/// The capsule center is at (0, 0, 0).
class CapsuleRepresentation : public virtual Representation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	/// \post	The capsule radius is 1.0.
	explicit CapsuleRepresentation(const std::string& name) : Representation(name)
	{
	}

	/// Sets the radius of the capsule
	/// \param	radius	Radius of the capsule
	virtual void setRadius(double radius) = 0;
	/// Returns the radius of the capsule
	/// \return Radius along X-axis and Z-axis of the capsule
	virtual double getRadius() const = 0;

	/// Sets the height of the capsulei
	/// \param	height	Height of the capsule
	virtual void setHeight(double height) = 0;
	/// Returns the height of the capsule
	/// \return Height along Y-axis of the capsule
	virtual double getHeight() const = 0;

	/// Sets the size of the capsule
	/// \param radius Size along X-axis and Z-axis of the capsule
	/// \param height Size along Y-axis of the capsule
	virtual void setSize(double radius, double height) = 0;
	/// Gets the size of the capsule
	/// \param [out] radius Variable to receive the size along X-axis and Z-axis of the capsule
	/// \param [out] height Variable to receive the size along Y-axis of the capsule
	virtual void getSize(double* radius, double* height) = 0;

	/// Sets the size of the capsule
	/// \param size Size of the capsule
	virtual void setSize(SurgSim::Math::Vector2d size) = 0;
	/// Returns the radius of the capsule
	/// \return Size of the capsule
	virtual SurgSim::Math::Vector2d getSize() const = 0;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_CAPSULEREPRESENTATION_H
