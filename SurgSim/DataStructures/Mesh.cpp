// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

#include "SurgSim/DataStructures/Mesh.h"

#include <SurgSim/Framework/Assert.h>

using SurgSim::DataStructures::Mesh;
using SurgSim::Math::Vector3d;

Mesh::Mesh()
{
}

Mesh::~Mesh()
{
}

void Mesh::setVertexPositions(const std::vector<Vector3d>& positions, bool doUpdate)
{
	SURGSIM_ASSERT(m_vertices.size() == positions.size()) << "Number of positions must match number of vertices.";

	for (unsigned int i = 0; i < m_vertices.size(); ++i)
	{
		m_vertices[i].position = positions[i];
	}

	if (doUpdate)
	{
		update();
	}
}

bool Mesh::operator==(const Mesh& mesh) const
{
	return (typeid(*this) == typeid(mesh)) && isEqual(mesh);
}

bool Mesh::operator!=(const Mesh& mesh) const
{
	return (typeid(*this) != typeid(mesh)) || ! isEqual(mesh);
}