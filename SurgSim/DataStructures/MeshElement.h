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

#ifndef SURGSIM_DATA_STRUCTURES_MESH_ELEMENT_H
#define SURGSIM_DATA_STRUCTURES_MESH_ELEMENT_H

#include <array>
#include <memory>

namespace SurgSim
{

namespace DataStructures
{

/// Element structure for meshes. Elements link vertices in the mesh and store some extra data.
/// \tparam	N	Number of vertices in the element
/// \tparam	Data	Type of extra data stored in the element
/// \sa	Mesh
template <unsigned int N, class Data>
struct MeshElement
{
	/// Constructor. Data is set to nullptr.
	/// \param	vertices	IDs of the N element vertices
	MeshElement(const std::array<unsigned int, N> vertices, Data data) :
		vertices(vertices),
		data(data)
	{
	}

	/// Element vertices.
	std::array<unsigned int, N> vertices;
	/// Extra element data.
	Data data;

	/// Compare the elements and return true if equal, false if not equal.
	friend bool operator==(const MeshElement<N, Data>& element1, const MeshElement<N, Data>& element2)
	{
		return element1.vertices == element2.vertices && element1.data == element2.data;
	}

	/// Compare the elements and return false if equal, true if not equal.
	friend bool operator!=(const MeshElement<N, Data>& element1, const MeshElement<N, Data>& element2)
	{
		return ! (element1 == element2);
	}
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_MESHELEMENT_H
