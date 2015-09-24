// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include "SurgSim/Particles/RandomMeshPointGenerator.h"

#include "SurgSim/Math/MeshShape.h"

using SurgSim::Math::Vector2d;
using SurgSim::Math::Vector3d;


namespace SurgSim
{
namespace Particles
{

Math::Vector3d RandomMeshPointGenerator::pointInShape(std::shared_ptr<Math::Shape> shape)
{
	SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) <<
		"PointGenerator does not support generating points in shape: "<< shape->getType();

	return Vector3d::Zero();
}

Math::Vector3d RandomMeshPointGenerator::pointOnShape(std::shared_ptr<Math::Shape> shape)
{
	Vector3d point;
	auto mesh = std::static_pointer_cast<Math::MeshShape>(shape);

	if (mesh->getNumTriangles() > 0)
	{
		auto& triangles = mesh->getTriangles();
		std::uniform_int_distribution<int> triangleSelector(0, triangles.size() - 1);

		bool validTriangleFound = false;
		size_t index;
		while (!validTriangleFound)
		{
			index = triangleSelector(m_generator);
			validTriangleFound = triangles[index].isValid;
		}
		auto vertices = mesh->getTrianglePositions(index);

		// Find a random point on the triangle using algorithm developed by Osada et al.
		//   R. Osada, T. Funkhouser, B. Chazelle, D. Dobkin, "Shape Distributions",
		//   ACM Transactions on Graphics, vol. 21, no. 4, pp. 807–832, October 2002
		Vector2d random = Vector2d::NullaryExpr([&](int index){return m_closedZeroOneDistribution(m_generator);});
		random[0] = sqrt(random[0]);
		point = (1 - random[0]) * vertices[0];
		point += random[0] * (1 - random[1]) * vertices[1];
		point += random[0] * random[1] * vertices[2];
	}
	else
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) <<
			"Mesh does not contain any triangles, cannot generate point.";
		point = Vector3d::Zero();
	}
	return point;
}


}; // namespace Particles
}; // namespace SurgSim

