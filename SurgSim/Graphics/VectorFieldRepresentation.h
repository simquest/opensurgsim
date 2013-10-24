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

namespace SurgSim
{
namespace Graphics
{

using SurgSim::DataStructures::Vertices;
using SurgSim::DataStructures::Vertex;
/// Graphic representation of a vector field
/// For each point, it is associated with a vector and an optional color
/// \tparam	Data Optional color information associated with each vector
template < class Data >
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

	/// Sets Vertices
	/// \param	vertices The Vertices (data structure)
	virtual void setVertices(std::shared_ptr< Vertices < Vertex<Data> > > vertices) = 0;
	/// Gets the Vertices (data structure)
	/// \return	The Vertices (data structure)
	virtual std::shared_ptr< Vertices< Vertex<Data> > > getVertices() const = 0;

	/// Sets vector line width
	/// \param	val	Width of vector line
	virtual void setLineWidth(double width) = 0;
	/// Gets line width
	/// \return	The line width
	virtual double getLineWidth() const = 0;
};

}; // Graphics
}; // SurgSim

#endif // SURGSIM_GRAPHICS_VECTORFIELDREPRESENTATION_H