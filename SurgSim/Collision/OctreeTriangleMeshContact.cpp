// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

#include "SurgSim/Collision/OctreeTriangleMeshContact.h"

using SurgSim::Math::MeshShape;


namespace SurgSim
{
namespace Collision
{

std::pair<int, int> OctreeTriangleMeshContact::getShapeTypes()
{
	return std::pair<int, int>(SurgSim::Math::SHAPE_TYPE_OCTREE, SurgSim::Math::SHAPE_TYPE_MESH);
}

std::list<std::shared_ptr<Contact>> OctreeTriangleMeshContact::boxContactCalculation(
		const SurgSim::Math::BoxShape& boxShape, const SurgSim::Math::RigidTransform3d& boxPose,
		const SurgSim::Math::Shape& otherShape, const SurgSim::Math::RigidTransform3d& otherPose)
{
	auto boxMesh = m_meshFactory.getInstance();

	static const std::array<std::array<size_t, 3>, 12> triangles = {{ {6, 7, 3}, {6, 3, 2}, {0, 4, 5}, {0, 5, 1},
		{1, 5, 6}, {1, 6, 2}, {0, 3, 7}, {0, 7, 4}, {4, 7, 6}, {4, 6, 5}, {0, 1, 2}, {0, 2, 3} }};

	if (boxMesh->getNumVertices() == 0)
	{
		boxMesh->getVertices().resize(8);
		boxMesh->getTriangles().reserve(12);
		for (const auto& triangle : triangles)
		{
			boxMesh->addTriangle(MeshShape::TriangleType(triangle));
		}
	}

	auto shapeVertex = boxShape.getVertices().cbegin();
	auto meshVertex = boxMesh->getVertices().begin();
	for( ; meshVertex != boxMesh->getVertices().end(); ++shapeVertex, ++meshVertex)
	{
		meshVertex->position = boxPose * (*shapeVertex);
	}
	boxMesh->update();

	return m_calculator.calculateDcdContact(*boxMesh, boxPose, static_cast<const MeshShape&>(otherShape), otherPose);
}

};
};
