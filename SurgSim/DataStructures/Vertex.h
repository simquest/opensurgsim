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

#ifndef SURGSIM_DATASTRUCTURES_VERTEX_H
#define SURGSIM_DATASTRUCTURES_VERTEX_H

#include "SurgSim/Math/Vector.h"

#include <memory>

namespace SurgSim
{

namespace DataStructures
{

/// Vertex structure for meshes. Vertices are the lowest level of structure in a Mesh, providing a position and can
/// store extra per-vertex data. MeshElements combine MeshVertices to form the structure of the mesh.
///
/// Vertex is to be used purely as a data structure and not provide implementation of algorithms.
/// For example, a physics FEM's nodes are not subclasses of Vertex if they provide code that is part of the FEM
/// algorithm, but they may used with a Mesh to store the structure of the FEM.
///
/// The extra Data is left up to the particular use of Mesh to specify. For example, for use collision detection,
/// a vertex may need a normal and adjacent triangle information, which could be stored in a struct.
///
/// If no extra Data is needed, a specialization exists for void, in which case the constructor takes no data.
///
/// \tparam	Data	Type of extra data stored in the vertex (void for no data)
/// \sa	Vertices
template <class Data>
struct Vertex
{
	/// Constructor
	/// \param	position	Position of the vertex
	/// \param	data	Extra data to be stored in the vertex
	explicit Vertex(const SurgSim::Math::Vector3d& position, const Data& data = Data()) :
		position(position),
		data(data)
	{
	}

	/// Copy constructor when the template data is a different type
	/// In this case, no data will be copied
	/// \tparam T type of data stored in the other Vertex
	/// \param other the Vertex to copy from
	template <class T>
	explicit Vertex(const Vertex<T>& other) :
		position(other.position)
	{
	}

	/// Position of the vertex.
	SurgSim::Math::Vector3d position;
	/// Extra vertex data.
	Data data;

	/// Compare the vertex to another one (equality)
	/// \param vertex The Vertex to compare it to
	/// \return True if the two vertices are equal, false otherwise.
	bool operator==(const Vertex<Data>& vertex) const
	{
		return data == vertex.data && position == vertex.position;
	}

	/// Compare the vertex to another one (inequality)
	/// \param vertex The Vertex to compare it to
	/// \return False if the two vertices are equal, true otherwise.
	bool operator!=(const Vertex<Data>& vertex) const
	{
		return ! ((*this) == vertex);
	}
};

/// Specialization of Vertex with no data.
/// \sa Vertex
template <>
struct Vertex<void>
{
	/// Constructor
	/// \param	position	Position of the vertex
	explicit Vertex(const SurgSim::Math::Vector3d& position) : position(position)
	{
	}

	/// Position of the vertex.
	SurgSim::Math::Vector3d position;

	/// Compare the vertex to another one (equality)
	/// \param vertex The Vertex to compare it to
	/// \return True if the two vertices are equal, false otherwise.
	bool operator==(const Vertex<void>& vertex) const
	{
		return position == vertex.position;
	}

	/// Compare the vertex to another one (inequality)
	/// \param vertex The Vertex to compare it to
	/// \return False if the two vertices are equal, true otherwise.
	bool operator!=(const Vertex<void>& vertex) const
	{
		return ! ((*this) == vertex);
	}
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_VERTEX_H
