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

#include <vector>

#include "SurgSim/DataStructures/MeshElement.h"
#include "SurgSim/DataStructures/TriangleMesh.h"

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
class TetrahedronMesh : public TriangleMesh<VertexData, EdgeData, TriangleData>
{
public:
	/// Tetrahedron type for convenience (Ids of the 4 vertices)
	typedef MeshElement<4, TetrahedronData> TetrahedronType;

	/// Constructor. The mesh is initially empty (no vertices, no edges, no triangles, no tetrahedrons).
	TetrahedronMesh();

	/// Destructor
	virtual ~TetrahedronMesh();

	/// Adds a tetrahedron to the mesh.
	/// \param	tetrahedron	Tetrahedron to add to the mesh
	/// Recommend that subclasses with a specific purpose (such as for use in collision detection) have a
	/// createTetrahedron(vertices, other data...) method which performs any checking desired and sets up the
	/// tetrahedron data based on the vertices and other parameters.
	/// \return	Unique ID of the new tetrahedron.
	size_t addTetrahedron(const TetrahedronType& tetrahedron);

	/// Returns the number of tetrahedrons in this mesh.
	/// \return The number of tetrahedrons
	size_t getNumTetrahedrons() const;

	/// Returns a vector containing the position of each tetrahedron.
	/// \return The vector containing all tetrahedrons
	const std::vector<TetrahedronType>& getTetrahedrons() const;
	/// Returns a vector containing the position of each tetrahedron (non const version).
	/// \return The vector containing all tetrahedrons
	std::vector<TetrahedronType>& getTetrahedrons();

	/// Returns the specified triangle.
	/// Returns the specified tetrahedron.
	/// \param id The tetrahedron's id
	/// \return The tetrahedron id
	/// \note No check is performed on the id
	const TetrahedronType& getTetrahedron(size_t id) const;

	/// Returns the specified tetrahedron (non const version).
	/// \param id The tetrahedron's id
	/// \return The tetrahedron id
	/// \note No check is performed on the id
	TetrahedronType& getTetrahedron(size_t id);

	/// Test if the TetrahedronMesh is valid (valid vertex Ids used in all MeshElements)
	/// \return True if the TetrahedronMesh is valid, False otherwise (the topology is then broken)
	bool isValid() const;

protected:
	/// Remove all tetrahedrons from the mesh.
	virtual void doClearTetrahedrons();

	using TriangleMesh<VertexData, EdgeData, TriangleData>::doClearVertices;
	using TriangleMesh<VertexData, EdgeData, TriangleData>::doClearEdges;
	using TriangleMesh<VertexData, EdgeData, TriangleData>::doClearTriangles;

	/// Internal comparison of meshes of the same type: returns true if equal, false if not equal.
	/// Override this method to provide custom comparison. Basic TriangleMesh implementation compares vertices,
	/// edges and triangles: the order of vertices, edges, and triangles must also match to be considered equal.
	/// \param	mesh	Mesh must be of the same type as that which it is compared against
	virtual bool isEqual(const Vertices<VertexData>& mesh) const;

private:

	/// Clear mesh to return to an empty state (no vertices, no edges, no triangles, no m_tetrahedrons).
	virtual void doClear();

	/// Tetrahedrons
	std::vector<TetrahedronType> m_tetrahedrons;
};

};  // namespace DataStructures

};  // namespace SurgSim

#include "SurgSim/DataStructures/TetrahedronMesh-inl.h"

#endif  // SURGSIM_DATASTRUCTURES_TETRAHEDRONMESH_H
