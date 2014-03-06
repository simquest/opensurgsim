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

#ifndef SURGSIM_DATASTRUCTURES_TRIANGLEMESHBASE_H
#define SURGSIM_DATASTRUCTURES_TRIANGLEMESHBASE_H

#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/DataStructures/MeshElement.h"

namespace SurgSim
{

namespace DataStructures
{

/// Basic class for storing Triangle Meshes, handling basic vertex, edge, and triangle functionality.
///
/// TriangleMeshBase is to be used purely as a data structure and not provide implementation of algorithms.
/// For example, a physics 2D FEM is not a subclass of TriangleMeshBase, but may use a TriangleMeshBase for storing the
/// structure of the FEM.
///
/// It is recommended that subclasses with a specific purpose (such as for use in collision detection) provide
/// convenience methods for creation of vertices, edges, and triangles and the data each contains. Methods such as
/// createVertex(position, other data...), createEdge(vertices, other data...),
/// and createTriangle(vertices, other data...) simplify the creation of vertices and elements and the data required.
/// These methods would use the addVertex(), addEdge(), and addTriangle() methods to add the created vertices and
/// elements to the TriangleMeshBase.
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
class TriangleMeshBase : public Vertices<VertexData>
{
public:
	/// Edge type for convenience  (Ids of the 2 vertices)
	typedef MeshElement<2, EdgeData> EdgeType;
	/// Triangle type for convenience  (Ids of the 3 vertices)
	typedef MeshElement<3, TriangleData> TriangleType;

	/// Constructor. The mesh is initially empty (no vertices, no edges, no triangles).
	TriangleMeshBase();

	// Copy constructor.
	template <class VertexDataSource, class EdgeDataSource, class TriangleDataSource>
	TriangleMeshBase(const TriangleMeshBase<VertexDataSource, EdgeDataSource, TriangleDataSource>& mesh);

	/// Destructor
	virtual ~TriangleMeshBase();

	/// Adds an edge to the mesh.
	/// No checking on the edge's vertices is performed.
	/// Recommend that subclasses with a specific purpose (such as for use in collision detection) have a
	/// createEdge(vertices, other data...) method which performs any checking desired and sets up the edge data based
	/// on the vertices and other parameters.
	/// \param	edge	Edge to add to the mesh
	/// \return	Unique ID of the new edge.
	unsigned int addEdge(const EdgeType& edge);

	/// Adds a triangle to the mesh.
	/// \param	triangle	Triangle to add to the mesh
	/// Recommend that subclasses with a specific purpose (such as for use in collision detection) have a
	/// createTriangle(vertices, other data...) method which performs any checking desired and sets up the triangle data
	/// based on the vertices and other parameters.
	/// \return	Unique ID of the new triangle.
	unsigned int addTriangle(const TriangleType& triangle);

	/// Get the number of edges
	/// Returns the number of edges in this mesh.
	unsigned int getNumEdges() const;

	/// Get the number of triangles
	/// Returns the number of triangles in this mesh.
	unsigned int getNumTriangles() const;

	/// Retrieve all edges
	/// Returns a vector containing the position of each edge.
	const std::vector<EdgeType>& getEdges() const;
	/// Retrieve all edges (non const version)
	/// Returns a vector containing the position of each edge.
	std::vector<EdgeType>& getEdges();

	/// Retrieve all triangles
	/// Returns a vector containing the position of each triangle.
	const std::vector<TriangleType>& getTriangles() const;
	/// Retrieve all triangles (non const version)
	/// Returns a vector containing the position of each triangle.
	std::vector<TriangleType>& getTriangles();

	/// Retrieve a specific edge
	/// Returns the specified edge.
	const EdgeType& getEdge(unsigned int id) const;
	/// Retrieve a specific edge (non const version)
	/// Returns the specified edge.
	EdgeType& getEdge(unsigned int id);

	/// Retrieve a specific triangle
	/// Returns the specified triangle.
	const TriangleType& getTriangle(unsigned int id) const;
	/// Retrieve a specific triangle (non const version)
	/// Returns the specified triangle.
	TriangleType& getTriangle(unsigned int id);

	/// Test if the TriangleMeshBase is valid (valid vertex Ids used in all MeshElements)
	/// \return True if the TriangleMeshBase is valid, False otherwise (the topology is then broken)
	bool isValid() const;

protected:
	/// Remove all edges from the mesh.
	virtual void doClearEdges();

	/// Remove all triangles from the mesh.
	virtual void doClearTriangles();

	/// Internal comparison of meshes of the same type: returns true if equal, false if not equal.
	/// Override this method to provide custom comparison. Basic TriangleMeshBase implementation compares vertices,
	/// edges and triangles: the order of vertices, edges, and triangles must also match to be considered equal.
	/// \param	mesh	Mesh must be of the same type as that which it is compared against
	/// \return True if the vertices are equals, False otherwise
	virtual bool isEqual(const Vertices<VertexData>& mesh) const;
private:

	/// Clear mesh to return to an empty state (no vertices, no edges, no triangles).
	virtual void doClear();

	/// Edges
	std::vector<EdgeType> m_edges;

	/// Triangles
	std::vector<TriangleType> m_triangles;

public:
	// Dependent name resolution for inherited functions and typenames from templates
	using typename Vertices<VertexData>::VertexType;
	using Vertices<VertexData>::addVertex;
};

};  // namespace DataStructures

};  // namespace SurgSim

#include "SurgSim/DataStructures/TriangleMeshBase-inl.h"

#endif  // SURGSIM_DATASTRUCTURES_TRIANGLEMESHBASE_H
