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

#ifndef SURGSIM_GRAPHICS_VECTORFIELDREPRESENTATION_H
#define SURGSIM_GRAPHICS_VECTORFIELDREPRESENTATION_H

#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Graphics/VectorField.h"

namespace SurgSim
{
namespace Graphics
{


/// Graphic representation of a vector field
/// Each point/location, i.e. (X,Y,Z), in the vector field is associated with a vector and an optional color
class VectorFieldRepresentation : public virtual Representation
{
public:
	/// Constructor
	/// \param name Name of VectorFieldRepresentation
	explicit VectorFieldRepresentation(const std::string& name) : Representation(name)
	{
	}

	/// Destructor
	virtual ~VectorFieldRepresentation()
	{
	}

	/// Gets the vector field
	/// \return The vector field
	virtual std::shared_ptr<SurgSim::Graphics::VectorField> getVectorField() const = 0;

	/// Updates the vector field in a threadsafe manner
	/// \param vectorfield the new data
	virtual void updateVectorField(const SurgSim::Graphics::VectorField& vectorfield) = 0;

	/// Sets vector line width
	/// \param	width	Width of vector line
	virtual void setLineWidth(double width) = 0;
	/// Gets line width
	/// \return	The line width
	virtual double getLineWidth() const = 0;

	/// Sets the scale to be applied to all vectors
	/// \param scale The scale
	virtual void setScale(double scale) = 0;
	/// Gets the scale applied to all vectors
	/// \return The scale
	virtual double getScale() const = 0;

	/// Sets the size of point indicating the starting of vector
	/// \param size	Size of starting point of a vector
	virtual void setPointSize(double size) = 0;
	/// Gets the size of starting point of a vector
	/// \return The size of starting point of a vector
	virtual	double getPointSize() const = 0;
};

}; // Graphics
}; // SurgSim

#endif // SURGSIM_GRAPHICS_VECTORFIELDREPRESENTATION_H