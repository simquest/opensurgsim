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

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/TriangleMeshBase.h"
#include "SurgSim/Framework/Asset.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Math
{
class MeshShape;
}

namespace DataStructures
{

/// Store normal for each triangle in a triangle mesh.
struct NormalData
{

	SurgSim::Math::Vector3d normal;

	/// Equality operator.
	/// \param	rhs	The right hand side NormalData.
	/// \return	true if the parameters are considered equivalent.
	bool operator==(const SurgSim::DataStructures::NormalData& rhs) const
	{
		return normal == rhs.normal;
	}

	/// Inequality operator.
	/// \param	rhs	The right hand side NormalData.
	/// \return	true if the parameters are not considered equivalent.
	bool operator!=(const SurgSim::DataStructures::NormalData& rhs) const
	{
		return !((*this) == rhs);
	}
};

typedef TriangleMeshBase<EmptyData, EmptyData, EmptyData> TriangleMeshPlain;

/// A TriangleMesh stores normal information for the triangles.
class TriangleMesh: public std::enable_shared_from_this<TriangleMesh>,
	public SurgSim::Framework::Asset,
	public SurgSim::DataStructures::TriangleMeshBase<EmptyData, EmptyData, NormalData>
{
public:

	/// Constructor
	TriangleMesh();

	SURGSIM_CLASSNAME(SurgSim::DataStructures::TriangleMesh);

	/// Templated constructor this lets us convert one mesh class into another mesh class
	/// \tparam	VertexDataSource	Type of extra data stored in each vertex
	/// \tparam	EdgeDataSource	Type of extra data stored in each edge
	/// \tparam	TriangleDataSource	Type of extra data stored in each triangle
	/// \param mesh The mesh to be copied from. Vertex, edge and triangle data will be emptied.
	/// \sa TriangleMeshBase
	template <class VertexDataSource, class EdgeDataSource, class TriangleDataSource>
	explicit TriangleMesh(const TriangleMeshBase<VertexDataSource, EdgeDataSource, TriangleDataSource>& mesh);

	/// Get normal for triangle.
	/// \param triangleId The triangle to get normal.
	/// \return The normal for the triangle with given ID.
	const SurgSim::Math::Vector3d& getNormal(size_t triangleId);

	/// Calculate normals for all triangles.
	/// \note Normals will be normalized.
	void calculateNormals();

	/// Sets the mesh's vertices and normals by transforming a similar mesh.  The two meshes must have the same number
	/// of vertices, edges, and triangles.
	/// \param pose the transformation to be applied to the vertices and norms
	/// \param source the mesh suppling the vertices and norms
	void copyWithTransform(const SurgSim::Math::RigidTransform3d& pose, const TriangleMesh& source);

protected:
	virtual void doUpdate() override;

	virtual bool doLoad(const std::string& fileName) override;

};

}; // namespace DataStructures
}; // namespace SurgSim

#include "SurgSim/DataStructures/TriangleMesh-inl.h"

#endif // SURGSIM_DATASTRUCTURES_TRIANGLEMESH_H
