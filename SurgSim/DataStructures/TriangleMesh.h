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

#ifndef SURGSIM_DATASTRUCTURES_TRIANGLEMESH_H
#define SURGSIM_DATASTRUCTURES_TRIANGLEMESH_H

#include <SurgSim/DataStructures/Vertices.h>
#include <SurgSim/DataStructures/MeshElement.h>

namespace SurgSim
{

namespace DataStructures
{

/// Basic class for storing Triangle Meshes, handling basic vertex, edge, and triangle functionality.
///
/// TriangleMesh is to be used purely as a data structure and not provide implementation of algorithms.
/// For example, a physics 2D FEM is not a subclass of TriangleMesh, but may use a TriangleMesh for storing the
/// structure of the FEM.
///
/// It is recommended that subclasses with a specific purpose (such as for use in collision detection) provide
/// convenience methods for creation of vertices, edges, and triangles and the data each contains. Methods such as
/// createVertex(position, other data...), createEdge(vertices, other data...),
/// and createTriangle(vertices, other data...) simplify the creation of vertices and elements and the data required.
/// These methods would use the addVertex(), addEdge(), and addTriangle() methods to add the created vertices and
/// elements to the TriangleMesh.
///
/// Overriding isEqual(const Mesh&) is necessary to do more than just basic list comparison of the
/// vertices, edges, and triangles, which is dependent on order in the list.
///
/// Override doUpdate() to provide update functionality when vertices are changes, such as recalculating surface
/// normals.
///
/// A subclass that is designed for a specific use (such as collision detection) may also specify the VertexData,
/// EdgeData, and TriangleData to what is required.
///
/// \tparam	VertexData	Type of extra data stored in each vertex
/// \tparam	EdgeData	Type of extra data stored in each edge
/// \tparam	TriangleData	Type of extra data stored in each triangle
/// \sa Vertex
/// \sa MeshElement
template <class VertexData, class EdgeData, class TriangleData>
class TriangleMesh : public Vertices<VertexData>
{
public:
	/// Edge type for convenience  (Ids of the 2 vertices)
	typedef MeshElement<2, EdgeData> EdgeType;
	/// Triangle type for convenience  (Ids of the 3 vertices)
	typedef MeshElement<3, TriangleData> TriangleType;

	/// Constructor. The mesh is initially empty (no vertices, no edges, no triangles).
	TriangleMesh()
	{
	}
	/// Destructor
	virtual ~TriangleMesh()
	{
	}

	/// Adds an edge to the mesh.
	/// No checking on the edge's vertices is performed.
	/// Recommend that subclasses with a specific purpose (such as for use in collision detection) have a
	/// createEdge(vertices, other data...) method which performs any checking desired and sets up the edge data based
	/// on the vertices and other parameters.
	/// \param	edge	Edge to add to the mesh
	/// \return	Unique ID of the new edge.
	unsigned int addEdge(const EdgeType& edge)
	{
		m_edges.push_back(edge);
		return m_edges.size() - 1;
	}

	/// Adds a triangle to the mesh.
	/// \param	triangle	Triangle to add to the mesh
	/// Recommend that subclasses with a specific purpose (such as for use in collision detection) have a
	/// createTriangle(vertices, other data...) method which performs any checking desired and sets up the triangle data
	/// based on the vertices and other parameters.
	/// \return	Unique ID of the new triangle.
	unsigned int addTriangle(const TriangleType& triangle)
	{
		m_triangles.push_back(triangle);
		return m_triangles.size() - 1;
	}

	/// Returns the number of edges in this mesh.
	unsigned int getNumEdges() const
	{
		return m_edges.size();
	}
	/// Returns the number of triangles in this mesh.
	unsigned int getNumTriangles() const
	{
		return m_triangles.size();
	}

	/// Returns a vector containing the position of each edge.
	const std::vector<EdgeType>& getEdges() const
	{
		return m_edges;
	}
	/// Returns a vector containing the position of each triangle.
	const std::vector<TriangleType>& getTriangles() const
	{
		return m_triangles;
	}

	/// Returns the specified edge.
	const EdgeType& getEdge(unsigned int id) const
	{
		return m_edges[id];
	}

	/// Returns the specified triangle.
	const TriangleType& getTriangle(unsigned int id) const
	{
		return m_triangles[id];
	}

	/// Test if the TriangleMesh is valid (valid vertex Ids used in all MeshElements)
	/// \return True if the TriangleMesh is valid, False otherwise (the topology is then broken)
	bool isValid() const
	{
		unsigned int numVertices = getNumVertices();

		// Test edges validity
		for (std::vector<EdgeType>::const_iterator it = m_edges.begin(); it != m_edges.end(); it++)
		{
			for (int vertexId = 0; vertexId < 2; vertexId++)
			{
				if (it->vertices[vertexId] >= numVertices)
				{
					return false;
				}
			}
		}

		// Test triangles validity
		for (std::vector<TriangleType>::const_iterator it = m_triangles.begin(); it != m_triangles.end(); it++)
		{
			for (int vertexId = 0; vertexId < 3; vertexId++)
			{
				if (it->vertices[vertexId] >= numVertices)
				{
					return false;
				}
			}
		}

		return true;
	}


protected:
	/// Remove all edges from the mesh.
	virtual void doClearEdges()
	{
		m_edges.clear();
	}
	/// Remove all triangles from the mesh.
	virtual void doClearTriangles()
	{
		m_triangles.clear();
	}

	/// Internal comparison of meshes of the same type: returns true if equal, false if not equal.
	/// Override this method to provide custom comparison. Basic TriangleMesh implementation compares vertices,
	/// edges and triangles: the order of vertices, edges, and triangles must also match to be considered equal.
	/// \param	mesh	Mesh must be of the same type as that which it is compared against
	virtual bool isEqual(const Vertices<VertexData>& mesh) const
	{
		const TriangleMesh& triangleMesh = static_cast<const TriangleMesh&>(mesh);
		return Vertices<VertexData>::isEqual(triangleMesh) && m_edges == triangleMesh.getEdges() &&
			m_triangles == triangleMesh.getTriangles();
	}
private:

	/// Clear mesh to return to an empty state (no vertices, no edges, no triangles).
	virtual void doClear()
	{
		doClearTriangles();
		doClearEdges();
		this->doClearVertices();
	}

	/// Edges
	std::vector<EdgeType> m_edges;
	/// Triangles
	std::vector<TriangleType> m_triangles;
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_TRIANGLEMESH_H
