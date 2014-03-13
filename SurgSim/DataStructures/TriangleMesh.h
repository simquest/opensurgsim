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
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace DataStructures
{

/// Store normal for each triangle in a triangle mesh.
struct NormalData {

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


/// A TriangleMesh stores normal information for the triangles.
class TriangleMesh: public SurgSim::DataStructures::TriangleMeshBase<EmptyData, EmptyData, NormalData>
{
public:
	/// Constructor
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
	SurgSim::Math::Vector3d getNormal(int triangleId);

	/// Calculate normals for all triangles.
	/// \note Normals will be normalized.
	void calculateNormals();
};

}; // namespace DataStructures
}; // namespace SurgSim

#include "SurgSim/DataStructures/TriangleMesh-inl.h"

#endif // SURGSIM_DATASTRUCTURES_TRIANGLEMESH_H
