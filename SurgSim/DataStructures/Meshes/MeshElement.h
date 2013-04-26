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

#include <SurgSim/DataStructures/Meshes/MeshElementData.h>

#include <array>
#include <memory>

namespace SurgSim
{

namespace DataStructures
{

template <unsigned int N>
class MeshElementData;

/// Edge structure for meshes.
/// \sa	Mesh
template <unsigned int N>
struct MeshElement
{
	/// Constructor. Data is set to nullptr.
	/// \param	vertices	IDs of the N element vertices
	MeshElement(const std::array<unsigned int, N> vertices);

	/// Constructor.
	/// \param	vertices	IDs of the N element vertices
	/// \param	data	Extra element data
	MeshElement(const std::array<unsigned int, N> vertices, std::shared_ptr<MeshElementData<N>> data);

	/// Element vertices.
	std::array<unsigned int, N> vertices;
	/// Extra element data.
	std::shared_ptr<MeshElementData<N>> data;

	/// Compare the elements and return true if equal, false if not equal.
	friend bool operator==(const MeshElement<N>& element1, const MeshElement<N>& element2)
	{
		bool isDataEqual = (element1.data == nullptr && element2.data == nullptr);
		if (element1.data != nullptr && element2.data != nullptr)
		{
			isDataEqual = *element1.data == *element2.data;
		}
		return element1.vertices == element2.vertices && isDataEqual;
	}

	/// Compare the elements and return false if equal, true if not equal.
	friend bool operator!=(const MeshElement<N>& element1, const MeshElement<N>& element2)
	{
		return ! (element1 == element2);
	}
};

};  // namespace DataStructures

};  // namespace SurgSim

#include "MeshElement-inl.h"

#endif  // SURGSIM_DATA_STRUCTURES_MESH_ELEMENT_DATA_H
