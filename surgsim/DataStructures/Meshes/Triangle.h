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

#ifndef SURGSIM_DATA_STRUCTURES_TRIANGLE_H
#define SURGSIM_DATA_STRUCTURES_TRIANGLE_H

#include <SurgSim/Math/Vector.h>

namespace SurgSim
{

namespace DataStructures
{

struct Triangle
{
	Triangle(unsigned int vertices[3], unsigned int edges[3]) : normal(0.0, 0.0, 0.0)
	{
		for (int i = 0; i < 3; ++i)
		{
			this->vertices[i] = vertices[i];
			this->edges[i] = edges[i];
		}
	}

	SurgSim::Math::Vector3d normal;

	// Defines the triangle points in counter-clockwise order
	unsigned int vertices[3];
	// Defines the edges as following:
	// edge0 = [vertex0, vertex1] NOT NECESSARILY IN THAT ORDER
	// edge1 = [vertex0, vertex2] NOT NECESSARILY IN THAT ORDER
	// edge2 = [vertex1, vertex2] NOT NECESSARILY IN THAT ORDER
	unsigned int edges[3];     

private:
	void updateNormal()
	{
		Vector3d u = vertices[1].position - vertices[0].position;
		Vector3d v = vertices[2].position - vertices[0].position;

		normal = u.cross(v);
		normal.normalize();
	}
};

}  // namespace DataStructures

}  // namespace SurgSim

#endif  // SURGSIM_DATA_STRUCTURES_TRIANGLE_H
