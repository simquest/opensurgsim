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

#include <array>
#include <memory>

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/MeshElement.h"
#include "SurgSim/DataStructures/NormalData.h"
#include "SurgSim/DataStructures/TriangleMeshPlyReaderDelegate.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Framework/Asset.h"

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
class TriangleMesh : public Vertices<VertexData>, public SurgSim::Framework::Asset,
	public std::enable_shared_from_this<TriangleMesh<VertexData, EdgeData, TriangleData>>
{
public:
	/// Edge type for convenience  (Ids of the 2 vertices)
	typedef MeshElement<2, EdgeData> EdgeType;
	/// Triangle type for convenience  (Ids of the 3 vertices)
	typedef MeshElement<3, TriangleData> TriangleType;

	/// Constructor. The mesh is initially empty (no vertices, no edges, no triangles).
	TriangleMesh();

	/// Copy constructor when the template data is the same type
	/// \param other the mesh to copy from
	TriangleMesh(const TriangleMesh<VertexData, EdgeData, TriangleData>& other);

	/// Copy constructor when the template data is a different type
	/// \tparam	V Type of extra data stored in each vertex
	/// \tparam	E Type of extra data stored in each edge
	/// \tparam	T Type of extra data stored in each triangle
	/// \param other The mesh to be copied from. Vertex, edge and triangle data will not be copied
	/// \note: Data of the input mesh, i.e. VertexDataSource, EdgeDataSource and TrianleDataSource will not be copied.
	template <class V, class E, class T>
	explicit TriangleMesh(const TriangleMesh<V, E, T>& other);

	/// Destructor
	virtual ~TriangleMesh();

	/// Move Constructor
	/// \param other Constructor source
	TriangleMesh(TriangleMesh&& other);

	/// Copy Assignment
	/// \param other Assignment source
	TriangleMesh<VertexData, EdgeData, TriangleData>& operator=(
		const TriangleMesh<VertexData, EdgeData, TriangleData>& other);

	/// Move Assignment
	/// \param other Assignment source
	TriangleMesh<VertexData, EdgeData, TriangleData>& operator=(
		TriangleMesh<VertexData, EdgeData, TriangleData>&& other);


	std::string getClassName() const override;

	/// Adds an edge to the mesh.
	/// No checking on the edge's vertices is performed.
	/// Recommend that subclasses with a specific purpose (such as for use in collision detection) have a
	/// createEdge(vertices, other data...) method which performs any checking desired and sets up the edge data based
	/// on the vertices and other parameters.
	/// \param	edge	Edge to add to the mesh
	/// \return	Unique ID of the new edge.
	size_t addEdge(const EdgeType& edge);

	/// Adds a triangle to the mesh.
	/// \param	triangle	Triangle to add to the mesh
	/// Recommend that subclasses with a specific purpose (such as for use in collision detection) have a
	/// createTriangle(vertices, other data...) method which performs any checking desired and sets up the triangle data
	/// based on the vertices and other parameters.
	/// \note The ids of deleted triangles will be reused in no particular order
	/// \return	id of the new triangle.
	size_t addTriangle(const TriangleType& triangle);

	/// Get the number of edges
	/// \return the number of edges in this mesh.
	size_t getNumEdges() const;

	/// Get the number of triangles
	/// \note The number of triangles might not match the size of the array returned by getTriangles(), after deletion
	///       has occurred it cannot be used to access all triangles.
	/// \return the number of triangles in this mesh.
	size_t getNumTriangles() const;

	/// Retrieve all edges
	/// \return a vector containing the position of each edge.
	const std::vector<EdgeType>& getEdges() const;

	/// Retrieve all edges (non const version)
	/// \return a vector containing the position of each edge.
	std::vector<EdgeType>& getEdges();

	/// Retrieve all triangles
	/// \note The number of triangles might not match the size of the array returned by getTriangles(), after deletion
	///       has occurred it cannot be used to access all triangles. When processing this array,
	///       check \sa TriangleType::isValid to see wether to do something with this triangle
	/// \return a vector containing the position of each triangle. Some of these triangles might be deleted, they need
	///         to be checked via \sa isValid before further processing
	const std::vector<TriangleType>& getTriangles() const;

	/// Retrieve all triangles (non const version)
	/// \note The number of triangles might not match the size of the array returned by getTriangles(), after deletion
	///       has occurred it cannot be used to access all triangles. When processing this array,
	///       check \sa TriangleType::isValid to see wether to do something with this triangle
	/// \return a vector containing the position of each triangle. Some of these triangles might be deleted, they need
	///         to be checked via \sa isValid before further processing
	std::vector<TriangleType>& getTriangles();

	/// Retrieve a specific edge
	/// \param id the edge to be retrieved.
	/// \return the specified edge.
	const EdgeType& getEdge(size_t id) const;

	/// Retrieve a specific edge (non const version)
	/// \param id the edge to be retrieved.
	/// \return the specified edge.
	EdgeType& getEdge(size_t id);

	/// Returns an array of the edge's vertices' positions
	/// \param id the id of the edge
	/// \return an array of the edge's vertices' positions
	std::array<SurgSim::Math::Vector3d, 2> getEdgePositions(size_t id) const;

	/// Retrieve a specific triangle
	/// \throws SurgSim::Framework::AssertionFailure if the given triangle was deleted
	/// \param id The id of the triangle to retrieve
	/// \return the specified triangle
	const TriangleType& getTriangle(size_t id) const;

	/// Retrieve a specific triangle (non const version)
	/// \throws SurgSim::Framework::AssertionFailure if the give triangle was deleted
	/// \param id The id of the triangle to retrieve
	/// \return the specified triangle
	TriangleType& getTriangle(size_t id);

	/// Marks a triangle as invalid, the triangle cannot be accessed via getTriangle anymore
	/// \note users of getTriangles() will have to check for deleted triangles if this feature is used
	///       the size of the vector returned by getTriangles does not reflect the number of triangles anymore
	///       use getNumTriangles() to figure out the correct number.
	/// \param id triangle to delete
	void removeTriangle(size_t id);

	/// Returns an array of the triangle's vertices' positions
	/// \param id the id of the triangle
	/// \return an array of the triangle's vertices' positions
	std::array<SurgSim::Math::Vector3d, 3> getTrianglePositions(size_t id) const;

	/// Test if the TriangleMesh is valid (valid vertex Ids used in all MeshElements)
	/// \return True if the TriangleMesh is valid, False otherwise (the topology is then broken)
	bool isValid() const;

	/// Save the triangle mesh in the ply format
	/// \param fileName the filename where to save
	void save(const std::string& fileName);

protected:
	/// Remove all edges from the mesh.
	virtual void doClearEdges();

	/// Remove all triangles from the mesh.
	virtual void doClearTriangles();

	/// Internal comparison of meshes of the same type: returns true if equal, false if not equal.
	/// Override this method to provide custom comparison. Basic TriangleMesh implementation compares vertices,
	/// edges and triangles: the order of vertices, edges, and triangles must also match to be considered equal.
	/// \param	mesh	Mesh must be of the same type as that which it is compared against
	/// \return True if the vertices are equals, False otherwise
	virtual bool isEqual(const Vertices<VertexData>& mesh) const;

	bool doLoad(const std::string& fileName) override;

	using Vertices<VertexData>::doClearVertices;

private:

	/// Clear mesh to return to an empty state (no vertices, no edges, no triangles).
	virtual void doClear();

	/// Edges
	std::vector<EdgeType> m_edges;

	/// Triangles
	std::vector<TriangleType> m_triangles;

	/// List of indices of deleted triangles, to be reused when another triangle is added
	std::vector<size_t> m_freeTriangles;

	static std::string m_className;

public:
	// Dependent name resolution for inherited functions and typenames from templates
	using typename Vertices<VertexData>::VertexType;
	using Vertices<VertexData>::addVertex;
	using Vertices<VertexData>::getNumVertices;
	using Vertices<VertexData>::getVertices;

};

typedef TriangleMesh<EmptyData, EmptyData, EmptyData> TriangleMeshPlain;

};  // namespace DataStructures
};  // namespace SurgSim

#include "SurgSim/DataStructures/TriangleMesh-inl.h"

#endif  // SURGSIM_DATASTRUCTURES_TRIANGLEMESH_H
