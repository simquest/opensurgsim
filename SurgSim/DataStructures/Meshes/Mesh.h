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

#ifndef SURGSIM_DATA_STRUCTURES_MESH_H
#define SURGSIM_DATA_STRUCTURES_MESH_H

#include <SurgSim/Math/Vector.h>

#include <array>
#include <vector>

namespace SurgSim
{

namespace DataStructures
{

class Mesh
{
public:
	/// Constructor initializes an empty mesh.
	Mesh();
	/// Destructor
	~Mesh();

	/// Reset to empty mesh.
	void reset()
	{
		doReset();
	}

	void update()
	{
		doUpdate();
	}

	// Return the new vertex ID
	unsigned int createNewVertex(const SurgSim::Math::Vector3d& position);

	unsigned int numVertices() const
	{ 
		m_vertexPositions.size();
	}
			
	const SurgSim::Math::Vector3d& getVertexPosition(unsigned int id) const
	{
		return m_vertexPositions[id];
	}

	void setVertexPositions(const std::vector<SurgSim::Math::Vector3d>& positions, bool doUpdate = false);
	void setVertexPositions(const Mesh &mesh, bool doUpdate = false);
	void setVertexPositions(const Mesh& mesh1, double percent1, const Mesh& mesh2, double percent2,
		bool doUpdate = false);

	const std::vector<SurgSim::Math::Vector3d>& getVertexPositions() const
	{
		return m_vertexPositions;
	}

	void loadFromParticles(std::string name, int nbPts, const double *pts);

protected:
	/// Reset to no vertices.
	virtual void doResetVertices()
	{
		m_vertexPositions.clear();
	}

private:
	/// Reset to empty mesh.
	virtual void doReset()
	{
		doResetVertices();
	}

	virtual void doUpdate()
	{
	}

	// Return -1 if the vertex does not already exist in m_vertices
	// Return the vertex ID if it already exist in m_vertices
	int doesThisVertexAlreadyExist(const double *v) const;

	/// Position of each vertex in the mesh.
	std::vector<SurgSim::Math::Vector3d> m_vertexPositions;
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif
