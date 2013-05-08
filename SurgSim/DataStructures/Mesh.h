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

#ifndef SURGSIM_DATASTRUCTURES_MESH_H
#define SURGSIM_DATASTRUCTURES_MESH_H

#include <SurgSim/DataStructures/MeshVertex.h>
#include <SurgSim/Framework/Assert.h>

#include <array>
#include <typeinfo>
#include <vector>

namespace SurgSim
{

namespace DataStructures
{

/// Base class for mesh structures, handling basic vertex functionality.
///
/// Mesh is to be used purely as a data structure and not provide implementation of algorithms.
/// For example, a physics FEM is not a subclass of Mesh, but may use a Mesh for storing the structure of the FEM.
///
/// Subclasses of this class should handle the elements required for a specific type of mesh (as simple as just a
/// generic triangle mesh or as specific as a triangle mesh for collision detection, which might also specify the data
/// types for the vertex and elements).
///
/// \tparam	VertexData	Type of extra data stored in each vertex (void for no data)
/// \sa MeshVertex
/// \sa MeshElement
template <class VertexData>
class Mesh
{
public:
	/// Vertex type for convenience
	typedef MeshVertex<VertexData> Vertex;

	/// Constructor. The mesh is initially empty (no vertices).
	Mesh()
	{
	}
	/// Destructor
	virtual ~Mesh()
	{
	}

	/// Clear mesh to return to an empty state (no vertices).
	void clear()
	{
		doClear();
	}

	/// Performs any updates that are required when the vertices are modified.
	/// Calls doUpdate() to perform the updates.
	void update()
	{
		doUpdate();
	}

	/// Adds a vertex to the mesh.
	/// \param	vertex	Vertex to add to the mesh
	/// \return	Unique ID of the new vertex.
	unsigned int addVertex(const Vertex& vertex)
	{
		m_vertices.push_back(vertex);
		return m_vertices.size() - 1;
	}

	/// Returns the number of vertices in this mesh.
	unsigned int getNumVertices() const
	{
		return m_vertices.size();
	}

	/// Returns the specified vertex.
	const Vertex& getVertex(unsigned int id) const
	{
		return m_vertices[id];
	}

	/// Returns a vector containing the position of each vertex.
	const std::vector<Vertex>& getVertices() const
	{
		return m_vertices;
	}

	/// Sets the position of a vertex.
	/// \param	id	Unique ID of the vertex
	/// \param	position	Position of the vertex
	void setVertexPosition(unsigned int id, const SurgSim::Math::Vector3d& position)
	{
		m_vertices[id].position = position;
	}
	/// Returns the position of a vertex.
	/// \param	id	Unique ID of the vertex
	/// \return	Position of the vertex
	const SurgSim::Math::Vector3d& getVertexPosition(unsigned int id) const
	{
		return m_vertices[id].position;
	}

	/// Sets the position of each vertex.
	/// \param	positions	Vector containing new position for each vertex
	/// \param	doUpdate	True to perform an update after setting the vertices, false to skip update; default is true.
	void setVertexPositions(const std::vector<SurgSim::Math::Vector3d>& positions, bool doUpdate = true)
	{
		SURGSIM_ASSERT(m_vertices.size() == positions.size()) << "Number of positions must match number of vertices.";

		for (unsigned int i = 0; i < m_vertices.size(); ++i)
		{
			m_vertices[i].position = positions[i];
		}

		if (doUpdate)
		{
			update();
		}
	}

	/// Compares the meshes and returns true if equal, false if not equal.
	bool operator==(const Mesh& mesh) const
	{
		return (typeid(*this) == typeid(mesh)) && isEqual(mesh);
	}

	/// Compares the meshes and returns false if equal, true if not equal.
	bool operator!=(const Mesh& mesh) const
	{
		return (typeid(*this) != typeid(mesh)) || ! isEqual(mesh);
	}

protected:
	/// Remove all vertices from the mesh.
	virtual void doClearVertices()
	{
		m_vertices.clear();
	}

	/// Internal comparison of meshes of the same type: returns true if equal, false if not equal.
	/// Override this method to provide custom comparison. Base Mesh implementation compares vertices:
	/// the order of vertices must also match to be considered equal.
	/// \param	mesh	Mesh must be of the same type as that which it is compared against
	virtual bool isEqual(const Mesh& mesh) const
	{
		return m_vertices == mesh.m_vertices;
	}

private:
	/// Clear mesh to return to an empty state (no vertices).
	virtual void doClear()
	{
		doClearVertices();
	}

	/// Performs any updates that are required when the vertices are modified.
	/// Override this method to implement update functionality.
	/// For example, this could be overridden to calculate normals for each MeshVertex.
	virtual void doUpdate()
	{
	}

	/// Vertices
	std::vector<Vertex> m_vertices;
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_MESH_H
