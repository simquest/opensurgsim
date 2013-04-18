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

/// Base class for mesh structures, handling basic vertex functionality.
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

	/// Performs any updates that are required when the vertices are modified.
	/// Calls doUpdate() to perform the updates.
	void update()
	{
		doUpdate();
	}

	/// Creates a new vertex.
	/// \param	position	Position of the vertex
	/// \return	Unique ID of the new vertex.
	unsigned int createNewVertex(const SurgSim::Math::Vector3d& position);

	/// Returns the number of vertices in this mesh.
	unsigned int numVertices() const
	{ 
		m_vertexPositions.size();
	}
	
	/// Returns the position of a vertex.
	/// \param	id	Unique ID of the vertex
	/// \return	Position of the vertex
	const SurgSim::Math::Vector3d& getVertexPosition(unsigned int id) const
	{
		return m_vertexPositions[id];
	}

	/// Sets the position of each vertex.
	/// \param	positions	Vector containing new position for each vertex
	/// \param	doUpdate	Whether or not to perform an update after setting the vertices
	void setVertexPositions(const std::vector<SurgSim::Math::Vector3d>& positions, bool doUpdate = false);
	/// Sets the position of each vertex to those of another mesh.
	/// \param	mesh	Mesh to copy vertex positions from
	/// \param	doUpdate	Whether or not to perform an update after setting the vertices
	void setVertexPositions(const Mesh& mesh, bool doUpdate = false);
	/// Sets the position of each vertex to the combination of two other meshes.
	/// \param	mesh1	First mesh to copy vertex positions from
	/// \param	percent1	Percent contribution of first mesh [0.0-1.0]
	/// \param	mesh2	Second mesh to copy vertex positions from
	/// \param	percent2	Percent contribution of second mesh [0.0-1.0]
	/// \param	doUpdate	Whether or not to perform an update after setting the vertices
	void setVertexPositions(const Mesh& mesh1, double percent1, const Mesh& mesh2, double percent2,
		bool doUpdate = false);

	/// Returns a vector containing the position of each vertex.
	const std::vector<SurgSim::Math::Vector3d>& getVertexPositions() const
	{
		return m_vertexPositions;
	}

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

	/// Performs any updates that are required when the vertices are modified.
	/// Override this method to implement update functionality.
	virtual void doUpdate()
	{
	}

	/// Position of each vertex in the mesh.
	std::vector<SurgSim::Math::Vector3d> m_vertexPositions;
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif
