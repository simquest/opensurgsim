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

#include "SurgSim/DataStructures/TriangleMesh.h"

namespace SurgSim
{
namespace DataStructures
{

SurgSim::Math::Vector3d TriangleMesh::getNormal(int triangleId) const
{
	return getTriangle(triangleId).data.normal;
}

void TriangleMesh::calculateNormals()
{
	for (size_t i = 0; i < getNumTriangles(); ++i)
	{
		const SurgSim::Math::Vector3d& vertex0 = getVertexPosition(getTriangle(i).verticesId[0]);
		const SurgSim::Math::Vector3d& vertex1 = getVertexPosition(getTriangle(i).verticesId[1]);
		const SurgSim::Math::Vector3d& vertex2 = getVertexPosition(getTriangle(i).verticesId[2]);

		// Calculate normal vector
		SurgSim::Math::Vector3d normal = (vertex1 - vertex0).cross(vertex2 - vertex0);
		normal.normalize();

		getTriangle(i).data.normal = normal;
	}
}

}; // namespace DataStructures
}; // namespace SurgSim
