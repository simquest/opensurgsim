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

#ifndef SURGSIM_DATASTRUCTURES_MESHELEMENT_H
#define SURGSIM_DATASTRUCTURES_MESHELEMENT_H

#include <array>
#include <memory>

namespace SurgSim
{

namespace DataStructures
{

/// Element structure for meshes. MeshElement combines Vertices to form the structure of a mesh and can store extra
/// per-element data.
///
/// MeshElement is to be used purely as a data structure and not provide implementation of algorithms.
/// For example, a physics FEM's elements are not subclasses of MeshElement if they provide code that is part of the FEM
/// algorithm, but they may used with a Mesh to store the structure of the FEM.
///
/// The extra Data is left up to the particular use of Mesh to specify. For example, for use collision detection,
/// a vertex may need a normal and adjacent triangle information, which could be stored in a struct.
///
/// If no extra Data is needed, a specialization exists for void, in which case the constructor takes no data.
///
/// \tparam	N	Number of vertices in the element
/// \tparam	Data	Type of extra data stored in the element (void for no data)
/// \sa	Vertices
template <unsigned int N, class Data>
struct MeshElement
{
	/// Constructor
	/// \param	verticesId IDs of the N element vertices
	/// \param	data	Extra data to be stored with the element
	MeshElement(const std::array<unsigned int, N>& verticesId, const Data& data) :
		verticesId(verticesId),
		data(data)
	{
	}

	/// Constructor with empty data member
	/// \param	verticesId IDs of the N element vertices
	MeshElement(const std::array<unsigned int, N>& verticesId) : 
		verticesId(verticesId)
		data()
	{
	}

	typedef std::array<unsigned int, N> IdType;

	/// Element vertices.
	IdType verticesId;
	/// Extra element data.
	Data data;

	/// Compare the element with another one (equality)
	/// \param element The MeshElement to compare it to
	/// \return True if the two MeshElements are equals, False otherwise
	bool operator==(const MeshElement<N, Data>& element) const
	{
		return verticesId == element.verticesId && data == element.data;
	}

	/// Compare the element with another one (inequality)
	/// \param element The MeshElement to compare it to
	/// \return False if the two MeshElements are equals, True otherwise
	bool operator!=(const MeshElement<N, Data>& element) const
	{
		return ! ((*this) == element);
	}
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_MESHELEMENT_H
