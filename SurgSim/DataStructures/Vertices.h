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

#ifndef SURGSIM_DATASTRUCTURES_VERTICES_H
#define SURGSIM_DATASTRUCTURES_VERTICES_H

#include <vector>

#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/Math/Vector.h"


namespace SurgSim
{

namespace DataStructures
{

/// Base class for mesh structures, handling basic vertex functionality.
///
/// Vertices is to be used purely as a data structure and not provide implementation of algorithms.
/// For example, a physics FEM is not a subclass of Vertices, but may use a Mesh for storing the structure of the FEM.
///
/// Subclasses of this class should handle the elements required for a specific type of mesh (as simple as just a
/// generic triangle mesh or as specific as a triangle mesh for collision detection, which might also specify the data
/// types for the vertex and elements).
///
/// It is recommended that subclasses of this class also provide convenience methods for creation of vertices and
/// elements, and the data each contains. A method such as createVertex(position, other data...) simplifies the creation
/// of vertices and the data required. This method would use the addVertex() method to add the created vertices to the
/// Mesh.
///
/// \tparam	VertexData	Type of extra data stored in each vertex (void for no data)
/// \sa Vertex
/// \sa MeshElement
template <class VertexData>
class Vertices
{
public:
	/// Vertex type for convenience
	typedef Vertex<VertexData> VertexType;

	/// Constructor
	Vertices();

	/// Copy constructor when the template data is a different type
	/// In this case, no data will be copied
	/// \tparam V type of data stored in the other Vertices
	/// \param other the Vertices to copy from
	template <class V>
	explicit Vertices(const Vertices<V>& other);

	/// Destructor
	virtual ~Vertices();

	/// Clear mesh to return to an empty state (no vertices).
	void clear();

	/// Performs any updates that are required when the vertices are modified.
	/// Calls doUpdate() to perform the updates.
	/// \return true on success.
	bool update();

	/// Adds a vertex to the mesh.
	/// Recommend that subclasses with a specific purpose (such as for use in collision detection), have a
	/// createVertex(position, other data...) method which performs any checking desired and sets up the vertex data
	/// based on the other parameters.
	/// \param	vertex	Vertex to add to the mesh
	/// \return	Unique ID of the new vertex.
	size_t addVertex(const VertexType& vertex);

	/// Returns the number of vertices in this mesh.
	size_t getNumVertices() const;

	/// Returns the specified vertex.
	const VertexType& getVertex(size_t id) const;

	/// Returns the specified vertex (non const version).
	VertexType& getVertex(size_t id);

	/// Returns a vector containing the position of each vertex.
	const std::vector<VertexType>& getVertices() const;

	/// Returns a vector containing the position of each vertex (non const version).
	std::vector<VertexType>& getVertices();

	/// Sets the position of a vertex.
	/// \param	id	Unique ID of the vertex
	/// \param	position	Position of the vertex
	void setVertexPosition(size_t id, const SurgSim::Math::Vector3d& position);

	/// Returns the position of a vertex.
	/// \param	id	Unique ID of the vertex
	/// \return	Position of the vertex
	const SurgSim::Math::Vector3d& getVertexPosition(size_t id) const;

	/// Sets the position of each vertex.
	/// \param	positions	Vector containing new position for each vertex
	/// \param	doUpdate	True to perform an update after setting the vertices, false to skip update; default is true.
	void setVertexPositions(const std::vector<SurgSim::Math::Vector3d>& positions, bool doUpdate = true);

	/// Compares the mesh with another one (equality)
	/// \param mesh The Vertices to compare it to
	/// \return True if the two vertices are equals, False otherwise
	bool operator==(const Vertices& mesh) const;

	/// Compares the mesh with another one (inequality)
	/// \param mesh The Vertices to compare it to
	/// \return False if the two vertices are equals, True otherwise
	bool operator!=(const Vertices& mesh) const;

protected:
	/// Remove all vertices from the mesh.
	virtual void doClearVertices();

	/// Internal comparison of meshes of the same type: returns true if equal, false if not equal.
	/// Override this method to provide custom comparison. Base Mesh implementation compares vertices:
	/// the order of vertices must also match to be considered equal.
	/// \param	mesh	Mesh must be of the same type as that which it is compared against
	virtual bool isEqual(const Vertices& mesh) const;

private:
	/// Clear mesh to return to an empty state (no vertices).
	virtual void doClear();

	/// Performs any updates that are required when the vertices are modified.
	/// Override this method to implement update functionality.
	/// For example, this could be overridden to calculate normals for each Vertex.
	/// \return true on success.
	virtual bool doUpdate();

	/// Vertices
	std::vector<VertexType> m_vertices;
};

};  // namespace DataStructures
};  // namespace SurgSim

#include "SurgSim/DataStructures/Vertices-inl.h"

#endif  // SURGSIM_DATASTRUCTURES_VERTICES_H
