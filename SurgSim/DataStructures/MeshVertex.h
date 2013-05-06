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

#include <SurgSim/DataStructures/MeshVertexData.h>
#include <SurgSim/Math/Vector.h>

#include <memory>

namespace SurgSim
{

namespace DataStructures
{

/// Vertex structure for meshes.
/// \sa	Mesh
struct MeshVertex
{
	/// Constructor. Data is set to nullptr.
	/// \param	position	Position of the vertex
	/// \param	data	Extra vertex data (default is null)
	MeshVertex(const SurgSim::Math::Vector3d& position, std::shared_ptr<MeshVertexData> data = nullptr);

	/// Position of the vertex.
	SurgSim::Math::Vector3d position;
	/// Extra vertex data.
	std::shared_ptr<MeshVertexData> data;

	/// Compare the vertices and return true if equal, false if not equal.
	friend bool operator==(const MeshVertex& vertex1, const MeshVertex& vertex2)
	{
		bool isDataEqual = (vertex1.data == nullptr && vertex2.data == nullptr);
		if (vertex1.data != nullptr && vertex2.data != nullptr)
		{
			 isDataEqual = *vertex1.data == *vertex2.data;
		}
		return vertex1.position == vertex2.position && isDataEqual;
	}

	/// Compare the vertices and return false if equal, true if not equal.
	friend bool operator!=(const MeshVertex& vertex1, const MeshVertex& vertex2)
	{
		return ! (vertex1 == vertex2);
	}
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATA_STRUCTURES_MESH_VERTEX_H
