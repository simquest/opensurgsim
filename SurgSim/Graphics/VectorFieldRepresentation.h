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

#include <SurgSim/DataStructures/Vertices.h>
#include <SurgSim/Graphics/Representation.h>
#include <SurgSim/Math/Vector.h>

namespace SurgSim
{
namespace Graphics
{

/// Graphic representation of a vector field
/// \tparam	Data Type associated with vertex.
template <class Data>
class VectorFieldRepresentation : public virtual Representation
{
public:
	/// Constructor
	explicit VectorFieldRepresentation(const std::string& name) : Representation(name)
	{
	}

	~VectorFieldRepresentation()
	{
	}

	/// Sets the vertices for the point cloud.
	/// \param	mesh	The mesh.
	virtual void setVertices(std::shared_ptr<SurgSim::DataStructures::Vertices<Data>> mesh) = 0;
	/// Pull the vertices.
	/// \return	The mesh.
	virtual std::shared_ptr<SurgSim::DataStructures::Vertices<Data>> getVertices() const = 0;

	/// Sets line width for the line elements.
	/// \param	width Width of the line
	virtual void setLineWidth(double width) = 0;
	/// Gets line width
	/// \return	The line width.
	virtual double getLineWidth() const = 0;

	/// Sets color for each vector in the vector field
	/// \param	color	The colors.
	virtual void setColors(const std::vector<SurgSim::Math::Vector4d>& colors) = 0;
	/// Gets the colors.
	/// \return The current colors.
	virtual std::vector<SurgSim::Math::Vector4d> getColors() const = 0;
};

}; // Graphics
}; // SurgSim

#endif // SURGSIM_GRAPHICS_VECTORFIELDREPRESENTATION_H