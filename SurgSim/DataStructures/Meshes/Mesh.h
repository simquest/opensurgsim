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

#ifndef SURGSIM_DATA_STRUCTURES_MESH_H
#define SURGSIM_DATA_STRUCTURES_MESH_H

#include <SurgSim/DataStructures/Meshes/MeshVertex.h>

#include <array>
#include <vector>

namespace SurgSim
{

namespace DataStructures
{

/// Base class for mesh structures, handling basic vertex functionality.
class Mesh
{
public:
	/// Vertex type
	typedef MeshVertex Vertex;

	/// Constructor. The mesh is initially empty (no vertices).
	Mesh();
	/// Destructor
	virtual ~Mesh();

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
	void setVertexPositions(const std::vector<SurgSim::Math::Vector3d>& positions, bool doUpdate = true);

	/// Compares the meshes and returns true if equal, false if not equal.
	bool operator==(const Mesh& mesh) const;

	/// Compares the meshes and returns false if equal, true if not equal.
	bool operator!=(const Mesh& mesh) const;

protected:
	/// Reset to no vertices.
	virtual void doClearVertices()
	{
		m_vertices.clear();
	}

	/// Internal comparison of meshes of the same type: returns true if equal, false if not equal.
	/// Override this method to provide custom comparison. Base implementation compares vertices.
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
	virtual void doUpdate()
	{
	}

	/// Vertices
	std::vector<Vertex> m_vertices;
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif
