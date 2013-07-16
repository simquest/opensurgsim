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

#ifndef SURGSIM_GRAPHICS_BOXREPRESENTATION_H
#define SURGSIM_GRAPHICS_BOXREPRESENTATION_H

#include <SurgSim/Graphics/Representation.h>

namespace SurgSim
{

namespace Graphics
{

/// Base graphics box representation class, which defines the basic interface for a box that can be visualized.
/// The box center is at (0, 0, 0).
class BoxRepresentation : public virtual Representation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	/// \post	The box size is (1.0,1.0,1.0).
	explicit BoxRepresentation(const std::string& name) : Representation(name)
	{
	}

	/// Sets the size along X-axis of the box
	/// \param sizeX Size along X-axis of the box
	virtual void setSizeX(double sizeX) = 0;
	/// Returns the size along X-axis of the box
	/// \return Size along X-axis of the box
	virtual double getSizeX() const = 0;

	/// Sets the size along Y-axis of the box
	/// \param sizeY Size along Y-axis of the box
	virtual void setSizeY(double sizeY) = 0;
	/// Returns the size along Y-axis of the box
	/// \return Size along Y-axis of the box
	virtual double getSizeY() const = 0;

	/// Sets the size along Z-axis of the box
	/// \param sizeZ Size along Z-axis of the box
	virtual void setSizeZ(double sizeZ) = 0;
	/// Returns the size along Z-axis of the box
	/// \return Size along Z-axis of the box
	virtual double getSizeZ() const = 0;

	/// Sets the size of the box
	/// \param sizeX Size along X-axis of the box
	/// \param sizeY Size along Y-axis of the box
	/// \param sizeZ Size along Z-axis of the box
	virtual void setSize(double sizeX, double sizeY, double sizeZ) = 0;
	/// Gets the size of the box
	/// \param sizeX Reference to store the size along X-axis of the box
	/// \param sizeY Reference to store the size along Y-axis of the box
	/// \param sizeZ Reference to store the size along Z-axis of the box
	virtual void getSize(double* sizeX, double* sizeY, double* sizeZ) = 0;

	/// Sets the size of the box
	/// \param size Size of the box
	virtual void setSize(SurgSim::Math::Vector3d size) = 0;
	/// Returns the radius of the sphere
	/// \return Size of the box
	virtual SurgSim::Math::Vector3d getSize() const = 0;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_BOXREPRESENTATION_H
