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

#ifndef SURGSIM_GRAPHICS_MESH_H
#define SURGSIM_GRAPHICS_MESH_H

#include <vector>

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/TriangleMeshBase.h"
#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Graphics
{

struct VertexData {
	SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector2d> texture;
	SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector4d> color;

	/// Equality operator.
	/// \param	rhs	The right hand side.
	/// \return	true if the parameters are considered equivalent.
	bool operator==(const SurgSim::Graphics::VertexData& rhs) const
	{
		return texture == rhs.texture &&
			   color == rhs.color;
	}

	/// Inequality operator.
	/// \param	rhs	The right hand side.
	/// \return	true if the parameters are not considered equivalent.
	bool operator!=(const SurgSim::Graphics::VertexData& rhs) const
	{
		return !((*this) == rhs);
	}
};

class Mesh : public SurgSim::DataStructures::TriangleMeshBase<VertexData, SurgSim::DataStructures::EmptyData,
					SurgSim::DataStructures::EmptyData>
{
public:
	/// Default constructor
	Mesh();

	/// Copy constructor
	/// \tparam	VertexDataSource	Type of extra data stored in each vertex
	/// \tparam	EdgeDataSource	Type of extra data stored in each edge
	/// \tparam	TriangleDataSource	Type of extra data stored in each triangle
	/// \param mesh The mesh to be copied from. Vertex, edge and triangle data will be emptied.
	/// \note: Data of the input mesh, i.e. VertexDataSource, EdgeDataSource and TrianleDataSource will not be copied.
	template <class VertexDataSource, class EdgeDataSource, class TriangleDataSource>
	explicit Mesh(const TriangleMeshBase<VertexDataSource, EdgeDataSource, TriangleDataSource>& mesh);

	/// Utility function to initialize a mesh with plain data,
	/// \param	vertices 	An array of vertex coordinates.
	/// \param	colors   	The colors, the number of colors can be 0 or
	/// 					there have to be at least as many colors as vertices.
	/// \param	textures 	The textures coordinates, the number of coordinates can be 0 or
	/// 					there have to be at least as many texture coordinates as there are vertices.
	/// \param	triangles	The triangles, a plain array of triplets of triangle indices, the indices should be
	/// 					points in the vertices array.
	void initialize(const std::vector<SurgSim::Math::Vector3d>& vertices,
					const std::vector<SurgSim::Math::Vector4d>& colors,
					const std::vector<SurgSim::Math::Vector2d>& textures,
					const std::vector<size_t>& triangles);
};

}; // Graphics
}; // SurgSim

#include "SurgSim/Graphics/Mesh-inl.h"

#endif // SURGSIM_GRAPHICS_MESH_H