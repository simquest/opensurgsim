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

#ifndef SURGSIM_DATA_STRUCTURES_VECTOR_H
#define SURGSIM_DATA_STRUCTURES_VECTOR_H

#include <SurgSim/Math/Vector.h>

#include <Vector>

namespace SurgSim
{

namespace DataStructures
{

struct Vertex
{
	Vertex(const SurgSim::Math::Vector3d& position) : position(position), normal(0.0, 0.0, 0.0)
	{
	}

	struct PositionComparator : public std::unary_function<Vertex, bool>
	{
		PositionComparator(const Vertex& baseline) : baseline(baseline) {}

		bool operator() (const Vertex& test)
		{ 
			return test.position == baseline.position; 
		}
		const Vertex& baseline;
	};

	SurgSim::Math::Vector3d position;
	SurgSim::Math::Vector3d normal;

	std::vector<unsigned int> edges;
	std::vector<unsigned int> triangles;

private:

	void updateNormal()
	{
		normal.setZero();

		// Do not count multiple triangles from the same face
		// Example: a rectangle decomposed to 2 triangles cause the same normal to be added twice for the same vertex.
		for (auto triangleIt = triangles.cbegin(); triangleIt != triangles.cend(); ++triangleIt)
		{
			const Vector3d& triangleNormal = m_triangles[*triangleIt].normal;

			// Check if the current triangle belongs to the same face as a triangle already added before.
			// If so, it is not valid, and its normal will not contribute to the vertex's normal.
			bool isTriangleValid = true;
			for (auto testIt = triangles.cbegin(); testIt != triangleIt; ++testIt)
			{
				const Vector3d& testNormal = m_triangles[*testIt].normal;

				// Same normal and the triangles share an edge => multiple triangles for a single face.
				if (triangleNormal.dot(testNormal) > 1.0 - 1e-4 &&
					sharesEdge(m_triangles[*triangleIt], m_triangles[*testIt]))
				{
					isTriangleValid = false;
				}
			}

			// If the triangle is valid, we add its normal component to the vertex normal.
			if (isTriangleValid)
			{
				normal += m_triangles[*triangleIt].normal;
			}
		}

		normal.normalize();
	}
};

}  // namespace DataStructures

}  // namespace SurgSim

#endif  // SURGSIM_DATA_STRUCTURES_VECTOR_H
