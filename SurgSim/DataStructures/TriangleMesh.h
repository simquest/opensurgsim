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

#include <SurgSim/DataStructures/Mesh.h>
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
/// Subclasses of this class can provide convenience methods for creation of vertices, edges, and triangles and the
/// data each contains. Overriding isEqual(const Mesh&) is necessary to do more than just basic list comparison of the
/// vertices, edges, and triangles, which is dependent on order in the list. Override doUpdate() to provide update
/// functionality when vertices are changes, such as recalculating surface normals. A subclass that is designed for a
/// specific use, such as collision detection, may also want to specify the VertexData, EdgeData, and TriangleData to
/// what is required.
///
/// \tparam	VertexData	Type of extra data stored in each vertex
/// \tparam	EdgeData	Type of extra data stored in each edge
/// \tparam	TriangleData	Type of extra data stored in each triangle
/// \sa MeshVertex
/// \sa MeshElement
template <class VertexData, class EdgeData, class TriangleData>
class TriangleMesh : public Mesh<VertexData>
{
public:
	/// Vertex type for convenience
	typedef typename MeshVertex<VertexData> Vertex;
	/// Edge type for convenience
	typedef typename MeshElement<2, EdgeData> Edge;
	/// Triangle type for convenience
	typedef typename MeshElement<3, TriangleData> Triangle;

	/// Constructor. The mesh is initially empty (no vertices, no edges, no triangles).
	TriangleMesh()
	{
	}
	/// Destructor
	virtual ~TriangleMesh()
	{
	}

	/// Adds an edge to the mesh.
	/// \param	edge	Edge to add to the mesh
	/// \return	Unique ID of the new edge.
	unsigned int addEdge(const Edge& edge)
	{
		m_edges.push_back(edge);
		return m_edges.size() - 1;
	}

	/// Adds an triangle to the mesh.
	/// \param	triangle	Triangle to add to the mesh
	/// \return	Unique ID of the new triangle.
	unsigned int addTriangle(const Triangle& triangle)
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
	const std::vector<Edge>& getEdges() const
	{
		return m_edges;
	}
	/// Returns a vector containing the position of each triangle.
	const std::vector<Triangle>& getTriangles() const
	{
		return m_triangles;
	}

	/// Returns the specified edge.
	const Edge& getEdge(unsigned int id) const
	{
		return m_edges[id];
	}

	/// Returns the specified triangle.
	const Triangle& getTriangle(unsigned int id) const
	{
		return m_triangles[id];
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
	virtual bool isEqual(const Mesh& mesh) const
	{
		const TriangleMesh& triangleMesh = static_cast<const TriangleMesh&>(mesh);
		return Mesh::isEqual(triangleMesh) && m_edges == triangleMesh.getEdges() &&
			m_triangles == triangleMesh.getTriangles();
	}
private:

	/// Clear mesh to return to an empty state (no vertices, no edges, no triangles).
	virtual void doClear()
	{
		doClearTriangles();
		doClearEdges();
		doClearVertices();
	}

	/// Edges
	std::vector<Edge> m_edges;
	/// Triangles
	std::vector<Triangle> m_triangles;
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_TRIANGLEMESH_H
