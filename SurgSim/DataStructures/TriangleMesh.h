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

#include "SurgSim/DataStructures/TriangleMeshBase.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace DataStructures
{

struct TriangleData {

	SurgSim::Math::Vector3d normal;

	/// Equality operator.
	/// \param	rhs	The right hand side.
	/// \return	true if the parameters are considered equivalent.
	bool operator==(const SurgSim::DataStructures::TriangleData& rhs) const
	{
		return normal == rhs.normal;
	}

	/// Inequality operator.
	/// \param	rhs	The right hand side.
	/// \return	true if the parameters are not considered equivalent.
	bool operator!=(const SurgSim::DataStructures::TriangleData& rhs) const
	{
		return !((*this) == rhs);
	}
};

// EmptyData class used for the TriangleMesh.
class EmptyData
{
public:
	bool operator==(const EmptyData& vertex) const
	{
		return true;
	}
};

class TriangleMesh: public SurgSim::DataStructures::TriangleMeshBase<EmptyData, EmptyData, TriangleData>
{
public:
	// Constructor
	template <class VertexDataSource, class EdgeDataSource, class TriangleDataSource>
	TriangleMesh(const std::shared_ptr<TriangleMeshBase<VertexDataSource, EdgeDataSource, TriangleDataSource>> mesh);

	// Get normal for triangle
	// \param triangleId The triangle to get normal
	SurgSim::Math::Vector3d getNormal(int triangleId);

	// Calculate normals for all triangles
	void calculateNormals();
};

}; // DataStructures
}; // SurgSim

#include "SurgSim/DataStructures/TriangleMesh-inl.h"

#endif // SURGSIM_DATASTRUCTURES_TRIANGLEMESH_H
