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

#include <SurgSim/Graphics/Representation.h>
#include <SurgSim/Graphics/VectorField.h>

namespace SurgSim
{
namespace Graphics
{

using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;

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

	/// Sets vector field
	/// \param	vertices	A vector field contains a list of vertices in 3D space
	/// \note Each vertex is associated with a vector and an optional color
	virtual void setVectorField(std::shared_ptr< SurgSim::Graphics::VectorField > vertices) = 0;
	/// Gets the vector field
	/// \return A vector field
	virtual std::shared_ptr< SurgSim::Graphics::VectorField > getVectorField() const = 0;

	/// Sets vector line width
	/// \param	width	Width of vector line
	virtual void setLineWidth(double width) = 0;
	/// Gets line width
	/// \return	The line width
	virtual double getLineWidth() const = 0;
};


}; // Graphics
}; // SurgSim

#endif // SURGSIM_GRAPHICS_VECTORFIELDREPRESENTATION_H