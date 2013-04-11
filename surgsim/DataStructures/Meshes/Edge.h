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

#ifndef SURGSIM_DATA_STRUCTURES_EDGE_H
#define SURGSIM_DATA_STRUCTURES_EDGE_H

#include <SurgSim/Math/Vector.h>

#include <unordered_set>

namespace SurgSim
{

namespace DataStructures
{

class Mesh;

class Edge
{
public:
	Edge(unsigned int vertex0, unsigned int vertex1) : m_normal(0.0, 0.0, 0.0)
	{
		m_vertices[0] = std::min(vertex0, vertex1);
		m_vertices[1] = std::max(vertex0, vertex1);
	}

	void addTriangle(unsigned int triangleId)
	{
		triangles.insert(triangleId);
	}

	void update(const Mesh& mesh)
	{
		doUpdate(mesh);
	}

private:

	virtual void doUpdate(const Mesh& mesh)
	{
		doUpdateNormal(mesh);
	}
	
	virtual void doUpdateNormal(const Mesh& mesh, const EdgeTopology& topology)
	{
		m_normal.setZero();

		for (auto triangleIt = topology.getTriangles().cbegin(); triangleIt != topology.getTriangles().cend(); ++triangleIt)
		{
			m_normal += mesh.getTriangle(*triangleIt).m_normal;
		}

		m_normal.normalize();
	}

	SurgSim::Math::Vector3d m_normal;

	std::array<unsigned int, 2> m_vertices;  // m_vertices[0] < m_vertices[1])
};

}  // namespace DataStructures

}  // namespace SurgSim

#endif  // SURGSIM_DATA_STRUCTURES_EDGE_H
