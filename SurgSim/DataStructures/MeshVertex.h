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

#ifndef SURGSIM_DATA_STRUCTURES_MESH_VERTEX_H
#define SURGSIM_DATA_STRUCTURES_MESH_VERTEX_H

#include <SurgSim/Math/Vector.h>

#include <memory>

namespace SurgSim
{

namespace DataStructures
{

/// Vertex structure for meshes. Vertices have a position and some extra data.
/// \tparam	Data	Type of extra data stored in the vertex
/// \sa	Mesh
template <class Data>
struct MeshVertex
{
	/// Constructor. Data is set to nullptr.
	/// \param	position	Position of the vertex
	/// \param	data	Extra vertex data (default is null)
	MeshVertex(const SurgSim::Math::Vector3d& position, Data data) :
		position(position),
		data(data)
	{
	}

	/// Position of the vertex.
	SurgSim::Math::Vector3d position;
	/// Extra vertex data.
	Data data;

	/// Compare the vertices and return true if equal, false if not equal.
	friend bool operator==(const MeshVertex<Data>& vertex1, const MeshVertex<Data>& vertex2)
	{
		return vertex1.data == vertex2.data && vertex1.position == vertex2.position;
	}

	/// Compare the vertices and return false if equal, true if not equal.
	friend bool operator!=(const MeshVertex<Data>& vertex1, const MeshVertex<Data>& vertex2)
	{
		return ! (vertex1 == vertex2);
	}
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATA_STRUCTURES_MESH_VERTEX_H
