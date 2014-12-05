// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

#ifndef SURGSIM_DATASTRUCTURES_UNITTESTS_MOCKOBJECTS_H
#define SURGSIM_DATASTRUCTURES_UNITTESTS_MOCKOBJECTS_H

#include "SurgSim/DataStructures/Grid.h"
#include "SurgSim/DataStructures/TetrahedronMesh.h"
#include "SurgSim/DataStructures/TriangleMeshBase.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Math/Vector.h"

#include <array>
#include <limits>

class ElementTest
{
public:
	int m_number;
	std::string m_string;

	ElementTest() : m_number(-1), m_string("Empty"){}

	explicit ElementTest(int i) : m_number(i) { std::stringstream s; s << i; m_string = s.str(); }

	ElementTest(const ElementTest& e) : m_number(e.m_number), m_string(e.m_string){}

	bool operator ==(const ElementTest& e) const
	{
		return m_number == e.m_number && m_string.compare(e.m_string) == 0;
	}
};

// Add a hash function for this class
namespace std
{
template <>
struct hash<ElementTest>
{
	std::size_t operator()(const ElementTest& k) const
	{
		using std::size_t;
		using std::hash;
		using std::string;

		// Compute individual hash values for first,
		// second and third and combine them using XOR
		// and bit shifting:
		return (hash<string>()(k.m_string) ^ (hash<int>()(k.m_number) << 1));
	}
};
}; // namespace std

template<typename T>
class Mock3DData
{
public:
	Mock3DData() :
		m_height(0),
		m_width(0),
		m_depth(0),
		m_buffer(0)
	{
	}

	Mock3DData(int height, int width, int depth) :
		m_height(height),
		m_width(width),
		m_depth(depth),
		m_buffer(height*depth*width)
	{
	}

	T get(int i, int j, int k)
	{
		return m_buffer[getIndex(i,j,k)];
	}

	void set(int i, int j, int k, T value)
	{
		m_buffer[getIndex(i,j,k)] = value;
	}

private:
	int getIndex(int i, int j, int k)
	{
		return i + j*m_width + k*m_width*m_height;
	}

	int m_height;
	int m_width;
	int m_depth;
	std::vector<T> m_buffer;
};

/// Vertex data for testing, storing ID and surface normal
class MockVertexData
{
public:
	/// Constructor
	/// \param	id	Unique ID of the vertex in its mesh
	/// \param	normal	Surface normal of the vertex
	MockVertexData(size_t id, const SurgSim::Math::Vector3d& normal) :
		m_id(id),
		m_normal(normal)
	{
	}
	/// Destructor
	virtual ~MockVertexData()
	{
	}

	/// Gets the vertex's unique ID in its mesh.
	size_t getId() const
	{
		return m_id;
	}

	/// Gets the vertex surface normal
	const SurgSim::Math::Vector3d& getNormal() const
	{
		return m_normal;
	}

	/// Compare the vertex data to another one (equality)
	/// \param data The MockVertexData to compare it to
	/// \return True if the two vertex data are equal, False otherwise
	bool operator==(const MockVertexData& data) const
	{
		return m_id == data.m_id && (m_normal - data.m_normal).norm() < 1.0e-10;
	}
private:
	/// Vertex's unique ID in its mesh
	size_t m_id;
	/// Vertex surface normal
	SurgSim::Math::Vector3d m_normal;
};

/// Edge data for testing, storing ID
class MockEdgeData
{
public:
	/// Constructor
	/// \param	id	Unique ID of the edge in its mesh
	explicit MockEdgeData(size_t id) :
		m_id(id)
	{
	}
	/// Destructor
	virtual ~MockEdgeData()
	{
	}

	/// Gets the edge's unique ID in its mesh.
	size_t getId() const
	{
		return m_id;
	}

	/// Compare the edge data to another one (equality)
	/// \param data The MockEdgeData to compare it to
	/// \return True if the two edge data are equal, False otherwise
	bool operator==(const MockEdgeData& data) const
	{
		return m_id == data.m_id;
	}
private:
	/// Edge's unique ID in its mesh
	size_t m_id;
};

/// Triangle data for testing, storing ID and edge IDs
class MockTriangleData
{
public:
	/// Constructor
	/// \param	id	Unique ID of the triangle in its mesh
	/// \param	edges	IDs of the triangle's edges in its mesh
	MockTriangleData(size_t id, const std::array<size_t, 3>& edges) :
		m_id(id),
		m_edges(edges)
	{
	}
	/// Destructor
	virtual ~MockTriangleData()
	{
	}

	/// Gets the triangle's unique ID in its mesh.
	size_t getId() const
	{
		return m_id;
	}

	/// Gets the IDs of the triangle's edges in its mesh.
	const std::array<size_t, 3>& getEdges() const
	{
		return m_edges;
	}

	/// Compare the triangle data to another one (equality)
	/// \param data The MockTriangleData to compare it to
	/// \return True if the two triangle data are equal, False otherwise
	bool operator==(const MockTriangleData& data) const
	{
		return m_id == data.m_id && m_edges == data.m_edges;
	}
private:
	/// Triangle's unique ID in its mesh
	size_t m_id;
	/// The IDs of the triangle's edges in its mesh, in order: {vertex0->vertex1, vertex1->vertex2, vertex2->vertex3}
	std::array<size_t, 3> m_edges;
};

/// Tetrahedron data for testing, storing ID and edge IDs, triangle IDs
class MockTetrahedronData
{
public:
	/// Constructor
	/// \param	id	Unique ID of the tetrahedron in its mesh
	/// \param	edges	IDs of the tetrahedron's edges in its mesh (6 edges)
	/// \param	triangles	IDs of the tetrahedron's triangles in its mesh (4 triangles)
	MockTetrahedronData(size_t id,
		const std::array<size_t, 6>& edges,
		const std::array<size_t, 4>& triangles) :
	  m_id(id), m_edges(edges), m_triangles(triangles)
	{
	}

	/// Destructor
	virtual ~MockTetrahedronData()
	{
	}

	/// Gets the tetrahedron's unique ID in its mesh.
	size_t getId() const
	{
		return m_id;
	}

	/// Gets the IDs of the tetrahedron's edges in its mesh.
	const std::array<size_t, 6>& getEdges() const
	{
		return m_edges;
	}

	/// Gets the IDs of the tetrahedron's triangles in its mesh.
	const std::array<size_t, 4>& getTriangles() const
	{
		return m_triangles;
	}

	/// Compare the tetrahedron data (equality)
	/// \param data The MockTetrahedronData to compare it to
	/// \return True if the two tetrahedron data are equals, False otherwise
	bool operator==(const MockTetrahedronData& data) const
	{
		return m_id == data.m_id && m_edges == data.m_edges && m_triangles == data.m_triangles;
	}
private:
	/// Tetrahedron's unique ID in its mesh
	size_t m_id;

	/// The IDs of the tetrahedron's edges in its mesh, in order:
	/// {vertex0->vertex1, vertex0->vertex2, vertex0->vertex3, vertex1->vertex2, vertex1->vertex3, vertex2->vertex3}
	std::array<size_t, 6> m_edges;

	/// The IDs of the tetrahedron's triangles in its mesh, in order: {vertex012, vertex123, vertex230, vertex301}
	std::array<size_t, 4> m_triangles;
};

/// Mesh for testing using MockVertexData
class MockMesh : public SurgSim::DataStructures::Vertices<MockVertexData>
{
public:
	/// Vertex type for convenience
	typedef Vertices<MockVertexData>::VertexType VertexType;

	/// Constructor. Start out with no vertices and 0 updates
	MockMesh() : SurgSim::DataStructures::Vertices<MockVertexData>(),
		m_numUpdates(0)
	{
	}
	/// Destructor
	virtual ~MockMesh()
	{
	}

	/// Create a new vertex in the mesh
	/// \param	position	Position of the vertex
	/// \param	normal	Normal of the vertex
	/// \return	Unique ID of vertex in the mesh
	size_t createVertex(const SurgSim::Math::Vector3d& position, const SurgSim::Math::Vector3d& normal)
	{
		VertexType vertex(position, MockVertexData(getNumVertices(), normal));

		return addVertex(vertex);
	}

	/// Returns the normal of a vertex
	const SurgSim::Math::Vector3d& getVertexNormal(size_t id) const
	{
		return getVertex(id).data.getNormal();
	}

	/// Returns the number of updates performed on the mesh
	int getNumUpdates() const
	{
		return m_numUpdates;
	}

private:
	/// Provides update functionality, which just increments the number of updates
	virtual void doUpdate()
	{
		++m_numUpdates;
	}

	/// Number of updates performed on the mesh
	int m_numUpdates;
};

/// Triangle Mesh for testing using MockVertexData, MockEdgeData, and MockTriangleData
class MockTriangleMeshBase : public SurgSim::DataStructures::TriangleMeshBase<MockVertexData,
																		  MockEdgeData,
																		  MockTriangleData>
{
public:
	/// Vertex type for convenience
	typedef TriangleMeshBase<MockVertexData, MockEdgeData, MockTriangleData>::VertexType VertexType;
	/// Edge type for convenience
	typedef TriangleMeshBase<MockVertexData, MockEdgeData, MockTriangleData>::EdgeType EdgeType;
	/// Triangle type for convenience
	typedef TriangleMeshBase<MockVertexData, MockEdgeData, MockTriangleData>::TriangleType TriangleType;

	/// Constructor. Start out with no vertices and 0 updates
	MockTriangleMeshBase() :SurgSim::DataStructures::TriangleMeshBase<MockVertexData, MockEdgeData, MockTriangleData>(),
		m_numUpdates(0)
	{
	}
	/// Destructor
	virtual ~MockTriangleMeshBase()
	{
	}

	/// Create a new vertex in the mesh
	/// \param	position	Position of the vertex
	/// \param	normal	Normal of the vertex
	/// \return	Unique ID of vertex in the mesh
	size_t createVertex(const SurgSim::Math::Vector3d& position, const SurgSim::Math::Vector3d& normal)
	{
		VertexType vertex(position, MockVertexData(getNumVertices(), normal));

		return addVertex(vertex);
	}

	/// Create a new edge in the mesh
	/// \param	vertices	Edge vertices
	/// \return	Unique ID of vertex in the mesh
	size_t createEdge(const std::array<size_t, 2>& vertices)
	{
		EdgeType edge(vertices, MockEdgeData(getNumEdges()));

		return addEdge(edge);
	}

	/// Create a new triangle in the mesh
	/// \param	vertices	The triangle vertices
	/// \param	edges	The triangle edges
	/// \return	Unique ID of vertex in the mesh
	size_t createTriangle(const std::array<size_t, 3>& vertices, const std::array<size_t, 3>& edges)
	{
		TriangleType triangle(vertices, MockTriangleData(getNumTriangles(), edges));

		return addTriangle(triangle);
	}

	/// Returns the normal of a vertex
	const SurgSim::Math::Vector3d& getVertexNormal(size_t id) const
	{
		return getVertex(id).data.getNormal();
	}

	/// Returns the number of updates performed on the mesh
	int getNumUpdates() const
	{
		return m_numUpdates;
	}

private:
	/// Provides update functionality, which just increments the number of updates
	virtual void doUpdate()
	{
		++m_numUpdates;
	}

	/// Number of updates performed on the mesh
	int m_numUpdates;
};

/// Tetrahedron Mesh for testing using MockVertexData, MockEdgeData, MockTriangleData and MockTetrahedronData
class MockTetrahedronMesh : public SurgSim::DataStructures::TetrahedronMesh<MockVertexData, MockEdgeData,
																		 MockTriangleData, MockTetrahedronData>
{
public:
	/// Vertex type for convenience
	typedef TetrahedronMesh<MockVertexData, MockEdgeData, MockTriangleData, MockTetrahedronData>::VertexType
		VertexType;
	/// Edge type for convenience
	typedef TetrahedronMesh<MockVertexData, MockEdgeData, MockTriangleData, MockTetrahedronData>::EdgeType
		EdgeType;
	/// Triangle type for convenience
	typedef TetrahedronMesh<MockVertexData, MockEdgeData, MockTriangleData, MockTetrahedronData>::TriangleType
		TriangleType;
	/// Tetrahedron type for convenience
	typedef TetrahedronMesh<MockVertexData, MockEdgeData, MockTriangleData, MockTetrahedronData>::TetrahedronType
		TetrahedronType;

	/// Constructor. Start out with no vertices and 0 updates
	MockTetrahedronMesh() :
		SurgSim::DataStructures::TetrahedronMesh<MockVertexData, MockEdgeData, MockTriangleData, MockTetrahedronData>(),
		m_numUpdates(0)
	{
	}
	/// Destructor
	virtual ~MockTetrahedronMesh()
	{
	}

	/// Create a new vertex in the mesh
	/// \param	position	Position of the vertex
	/// \param	normal	Normal of the vertex
	/// \return	Unique ID of vertex in the mesh
	size_t createVertex(const SurgSim::Math::Vector3d& position, const SurgSim::Math::Vector3d& normal)
	{
		VertexType vertex(position, MockVertexData(getNumVertices(), normal));

		return addVertex(vertex);
	}

	/// Create a new edge in the mesh
	/// \param	vertices	Edge vertices (x2)
	/// \return	Unique ID of vertex in the mesh
	size_t createEdge(const std::array<size_t, 2>& vertices)
	{
		EdgeType edge(vertices, MockEdgeData(getNumEdges()));

		return addEdge(edge);
	}

	/// Create a new triangle in the mesh
	/// \param	vertices (x3)
	/// \param edges (x3)
	/// \return	Unique ID of vertex in the mesh
	size_t createTriangle(const std::array<size_t, 3>& vertices, const std::array<size_t, 3>& edges)
	{
		TriangleType triangle(vertices, MockTriangleData(getNumTriangles(), edges));

		return addTriangle(triangle);
	}

	/// Create a new tetrahedron in the mesh
	/// \param	vertices connectivity (x4)
	/// \param	edges connectivity (x6)
	/// \param	triangles connectivity (x4)
	/// \return	Unique ID of vertex in the mesh
	size_t createTetrahedron(const std::array<size_t, 4>& vertices,
		const std::array<size_t, 6>& edges,
		const std::array<size_t, 4>& triangles)
	{
		TetrahedronType tetrahedron(vertices, MockTetrahedronData(getNumTetrahedrons(), edges, triangles));

		return addTetrahedron(tetrahedron);
	}

	/// Returns the normal of a vertex
	const SurgSim::Math::Vector3d& getVertexNormal(size_t id) const
	{
		return getVertex(id).data.getNormal();
	}

	/// Returns the number of updates performed on the mesh
	int getNumUpdates() const
	{
		return m_numUpdates;
	}

private:
	/// Provides update functionality, which just increments the number of updates
	virtual void doUpdate()
	{
		++m_numUpdates;
	}

	/// Number of updates performed on the mesh
	int m_numUpdates;
};

template <typename T, size_t N>
class MockGrid : public SurgSim::DataStructures::Grid<T, N>
{
public:
	MockGrid(const Eigen::Matrix<double, N, 1>& cellSize, const Eigen::AlignedBox<double, N>& bounds) :
		SurgSim::DataStructures::Grid<T,N>(cellSize, bounds)
	{
	}

	std::unordered_map<size_t, typename SurgSim::DataStructures::Grid<T, N>::CellContent>& getActiveCells()
	{
		return this->m_activeCells;
	}

	std::unordered_map<T, size_t>& getCellIds() { return this->m_cellIds; }

	std::vector<T>& getNonConstNeighbors(const T& element)
	{
		return const_cast<std::vector<T>&>(this->getNeighbors(element));
	}

	Eigen::Matrix<double, N, 1> getSize() const { return this->m_size; }

	Eigen::Matrix<size_t, N, 1> getNumCells() const { return this->m_numCells; }

	Eigen::Matrix<size_t, N, 1> getExponents() const { return this->m_exponents; }

	Eigen::Matrix<size_t, N, 1> getOffsetExponents() const { return this->m_offsetExponents; }

	Eigen::AlignedBox<double, N> getAABB() const { return this->m_aabb; }
};

namespace wrappers
{
template <typename IteratorType>
class formatIterator
{
public:
	formatIterator(IteratorType begin, IteratorType end, const std::string& separator)
		: m_begin(begin), m_end(end), m_separator(separator)
	{
	}

	friend std::ostream& operator<<(std::ostream& stream, const formatIterator& formatIterator)
	{
		if (formatIterator.m_begin == formatIterator.m_end)
		{
			return stream;
		}

		IteratorType start = formatIterator.m_begin;
		IteratorType penultimate = formatIterator.m_end - 1;
		while (start != penultimate)
		{
			stream << *start++ << formatIterator.m_separator;
		}
		stream << *start;

		return stream;
	}

private:
	IteratorType m_begin;
	IteratorType m_end;
	const std::string& m_separator;
};
}

template <typename IterableType>
wrappers::formatIterator<typename IterableType::const_iterator> formatIterator(const IterableType& iterable,
																			   const std::string& separator)
{
	return wrappers::formatIterator<typename IterableType::const_iterator>(
		iterable.cbegin(), iterable.cend(), separator);
}

template <typename IteratorType>
wrappers::formatIterator<IteratorType> formatIterator(IteratorType begin,
													  IteratorType end,
													  const std::string& separator)
{
	return wrappers::formatIterator<IteratorType>(begin, end, separator);
}

#endif  // SURGSIM_DATASTRUCTURES_UNITTESTS_MOCKOBJECTS_H
