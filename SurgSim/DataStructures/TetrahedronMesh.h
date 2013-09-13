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

#ifndef SURGSIM_DATASTRUCTURES_TETRAHEDRONMESH_H
#define SURGSIM_DATASTRUCTURES_TETRAHEDRONMESH_H

#include <SurgSim/DataStructures/TriangleMesh.h>

namespace SurgSim
{

namespace DataStructures
{

/// Basic class for storing Tetrahedron Meshes, handling basic vertex, edge, triangle and tetrahedron functionality.
///
/// TetrahedronMesh is to be used purely as a data structure and not provide implementation of algorithms.
/// For example, a physics 3D FEM is not a subclass of TetrahedronMesh, but may use a TetrahedronMesh for storing the
/// structure of the FEM.
///
/// It is recommended that subclasses with a specific purpose (such as for use in collision detection) provide
/// convenience methods for creation of vertices, edges, triangles, tetrahedrons and the data each contains.
/// Methods such as createVertex(position, other data...), createEdge(vertices, other data...),
/// createTriangle(vertices, other data...) and createTetrahedron(vertices, other data...) simplify the creation of
/// vertices and elements and the data required.
/// These methods would use the addVertex(), addEdge(), addTriangle() and addTetrahedron() methods to add the created
/// vertices and elements to the TetrahedronMesh.
///
/// Overriding isEqual(const Mesh&) is necessary to do more than just basic list comparison of the
/// vertices, edges, triangles and tetrahedrons which is dependent on order in the list.
///
/// Override doUpdate() to provide update functionality when vertices are changed, such as recalculating surface
/// normals.
///
/// A subclass that is designed for a specific use (such as collision detection) may also specify the VertexData,
/// EdgeData, TriangleData and TetrahedronData to what is required.
///
/// \tparam	VertexData	Type of extra data stored in each vertex
/// \tparam	EdgeData	Type of extra data stored in each edge
/// \tparam	TriangleData	Type of extra data stored in each triangle
/// \tparam	TetrahedronData	Type of extra data stored in each tetrahedron
/// \sa Vertices
/// \sa MeshElement
template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
class TetrahedronMesh : public Vertices<VertexData>
{
public:
	/// Edge type for convenience (Ids of the 2 vertices)
	typedef MeshElement<2, EdgeData> EdgeType;
	/// Triangle type for convenience (Ids of the 3 vertices)
	typedef MeshElement<3, TriangleData> TriangleType;
	/// Tetrahedron type for convenience (Ids of the 4 vertices)
	typedef MeshElement<4, TetrahedronData> TetrahedronType;

	/// Constructor. The mesh is initially empty (no vertices, no edges, no triangles, no tetrahedrons).
	TetrahedronMesh();

	/// Destructor
	virtual ~TetrahedronMesh();

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

	/// Adds a tetrahedron to the mesh.
	/// \param	tetrahedron	Tetrahedron to add to the mesh
	/// Recommend that subclasses with a specific purpose (such as for use in collision detection) have a
	/// createTetrahedron(vertices, other data...) method which performs any checking desired and sets up the
	/// tetrahedron data based on the vertices and other parameters.
	/// \return	Unique ID of the new tetrahedron.
	unsigned int addTetrahedron(const TetrahedronType& tetrahedron);

	/// Returns the number of edges in this mesh.
	/// \return The number of edges
	unsigned int getNumEdges() const;

	/// Returns the number of triangles in this mesh.
	/// \return The number of triangles
	unsigned int getNumTriangles() const;

	/// Returns the number of tetrahedrons in this mesh.
	/// \return The number of tetrahedrons
	unsigned int getNumTetrahedrons() const;

	/// Returns a vector containing the position of each edge.
	/// \return The vector containing all edges
	const std::vector<EdgeType>& getEdges() const;

	/// Returns a vector containing the position of each triangle.
	/// \return The vector containing all triangles
	const std::vector<TriangleType>& getTriangles() const;

	/// Returns a vector containing the position of each tetrahedron.
	/// \return The vector containing all tetrahedrons
	const std::vector<TetrahedronType>& getTetrahedrons() const;

	/// Returns the specified edge.
	/// \param id The edge's id
	/// \return The edge id
	/// \note No check is performed on the id
	const EdgeType& getEdge(unsigned int id) const;
	/// Returns the specified edge (non const version).
	/// \param id The edge's id
	/// \return The edge id
	/// \note No check is performed on the id
	EdgeType& getEdge(unsigned int id);

	/// Returns the specified triangle.
	/// \param id The triangle's id
	/// \return The triangle id
	/// \note No check is performed on the id
	const TriangleType& getTriangle(unsigned int id) const;

	/// Returns the specified tetrahedron.
	/// \param id The tetrahedron's id
	/// \return The tetrahedron id
	/// \note No check is performed on the id
	const TetrahedronType& getTetrahedron(unsigned int id) const;

	/// Test if the TetrahedronMesh is valid (valid vertex Ids used in all MeshElements)
	/// \return True if the TetrahedronMesh is valid, False otherwise (the topology is then broken)
	bool isValid() const;

protected:
	/// Remove all edges from the mesh.
	virtual void doClearEdges();

	/// Remove all triangles from the mesh.
	virtual void doClearTriangles();

	/// Remove all tetrahedrons from the mesh.
	virtual void doClearTetrahedrons();

	/// Internal comparison of meshes of the same type: returns true if equal, false if not equal.
	/// Override this method to provide custom comparison. Basic TriangleMesh implementation compares vertices,
	/// edges and triangles: the order of vertices, edges, and triangles must also match to be considered equal.
	/// \param	mesh	Mesh must be of the same type as that which it is compared against
	virtual bool isEqual(const Vertices<VertexData>& mesh) const;
private:

	/// Clear mesh to return to an empty state (no vertices, no edges, no triangles, no m_tetrahedrons).
	virtual void doClear();

	/// Edges
	std::vector<EdgeType> m_edges;

	/// Triangles
	std::vector<TriangleType> m_triangles;

	/// Tetrahedrons
	std::vector<TetrahedronType> m_tetrahedrons;
};

};  // namespace DataStructures

};  // namespace SurgSim

#include <SurgSim/DataStructures/TetrahedronMesh-inl.h>

#endif  // SURGSIM_DATASTRUCTURES_TETRAHEDRONMESH_H
