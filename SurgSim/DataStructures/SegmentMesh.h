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

#ifndef SURGSIM_DATASTRUCTURES_SEGMENTMESH_H
#define SURGSIM_DATASTRUCTURES_SEGMENTMESH_H

#include "SurgSim/DataStructures/SegmentEmptyData.h"
#include "SurgSim/DataStructures/TriangleMesh.h"

namespace SurgSim
{
namespace DataStructures
{

/// Class to hold the type of a SegmentMesh.
///
/// \tparam	VertexData	Type of extra data stored in each vertex
/// \tparam	EdgeData	Type of extra data stored in each edge
/// \sa TriangleMesh
template <class VertexData, class EdgeData>
class SegmentMesh : public TriangleMesh<VertexData, EdgeData, SegmentEmptyData>
{
public:
	/// TriangleMesh type for convenience
	typedef TriangleMesh<VertexData, EdgeData, SegmentEmptyData> TriangleMeshType;
	/// Edge type for convenience  (Ids of the 2 vertices)
	typedef typename TriangleMeshType::EdgeType EdgeType;
	/// Triangle type for convenience  (Ids of the 3 vertices)
	typedef typename TriangleMeshType::TriangleType TriangleType;

	/// Constructor. The mesh is initially empty (no vertices, no edges).
	SegmentMesh();

	/// Copy constructor when the template data is the same type
	/// \param other the mesh to copy from
	SegmentMesh(const SegmentMesh<VertexData, EdgeData>& other);

	/// Copy constructor when the template data is a different type
	/// \tparam	V Type of extra data stored in each vertex
	/// \tparam	E Type of extra data stored in each edge
	/// \param other The mesh to be copied from. Vertex and edge data will not be copied
	/// \note: Data of the input mesh, i.e. VertexDataSource, EdgeDataSource will not be copied.
	template <class V, class E>
	explicit SegmentMesh(const SegmentMesh<V, E>& other);

	/// Destructor
	virtual ~SegmentMesh();

	/// Move Constructor
	/// \param other Constructor source
	SegmentMesh(SegmentMesh&& other);

	/// Copy Assignment
	/// \param other Assignment source
	SegmentMesh<VertexData, EdgeData>& operator=(
		const SegmentMesh<VertexData, EdgeData>& other);

	/// Move Assignment
	/// \param other Assignment source
	SegmentMesh<VertexData, EdgeData>& operator=(
		SegmentMesh<VertexData, EdgeData>&& other);

	/// Creates edges for all vertices in the mesh connecting all the points consecutively
	/// \note This will clear all the current edges
	void createDefaultEdges();

	/// Save the current structure to a ply file
	/// \param filename Name of the file for writing
	/// \param asPhysics Format the file to be used as a physics file
	void save(const std::string& fileName, bool asPhyics = true);

	///@{
	/// Functions that need to assert, because they deal with triangles.
	size_t addTriangle(const TriangleType& triangle);
	size_t getNumTriangles() const;
	const std::vector<TriangleType>& getTriangles() const;
	std::vector<TriangleType>& getTriangles();
	const TriangleType& getTriangle(size_t id) const;
	TriangleType& getTriangle(size_t id);
	void removeTriangle(size_t id);
	std::array<SurgSim::Math::Vector3d, 3> getTrianglePositions(size_t id) const;
	void doClearTriangles() override;
	///@}

private:
	/// Clear mesh to return to an empty state (no vertices, no edges).
	void doClear() override;
};

typedef SegmentMesh<EmptyData, EmptyData> SegmentMeshPlain;

}  // namespace DataStructures
}  // namespace SurgSim

#include "SurgSim/DataStructures/SegmentMesh-inl.h"

#endif  // SURGSIM_DATASTRUCTURES_SEGMENTMESH_H
