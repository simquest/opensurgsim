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

#ifndef SURGSIM_DATA_STRUCTURES_UNITTESTS_MOCK_OBJECTS_H
#define SURGSIM_DATA_STRUCTURES_UNITTESTS_MOCK_OBJECTS_H

#include "SurgSim/DataStructures/Mesh.h"
#include "SurgSim/DataStructures/MeshElementData.h"
#include "SurgSim/DataStructures/MeshVertexData.h"
#include "SurgSim/Math/Vector.h"

#include <array>
#include <limits>

/// Vertex data for testing, storing ID and surface normal
class MockVertexData : public SurgSim::DataStructures::MeshVertexData
{
public:
	/// Constructor
	/// \param	id	Unique ID of the vertex in its mesh
	/// \param	normal	Surface normal of the vertex
	MockVertexData(unsigned int id, const SurgSim::Math::Vector3d& normal) : MeshVertexData(), 
		m_id(id), 
		m_normal(normal)
	{
	}
	/// Destructor
	virtual ~MockVertexData()
	{
	}

	/// Gets the vertex's unique ID in its mesh.
	unsigned int getId() const
	{
		return m_id;
	}

	/// Gets the vertex surface normal
	const SurgSim::Math::Vector3d& getNormal() const
	{
		return m_normal;
	}

private:
	/// Vertex's unique ID in its mesh
	const unsigned int m_id;
	/// Vertex surface normal
	const SurgSim::Math::Vector3d m_normal;

	/// Internal comparison of data of the same type: returns true if equal, false if not equal.
	/// Compares edge ID and normal.
	/// \param	data	Data must be of the same type as that which it is compared against
	virtual bool isEqual(const MeshVertexData& data) const
	{
		const MockVertexData& castData = static_cast<const MockVertexData&>(data);
		return m_id == castData.m_id && (m_normal - castData.m_normal).norm() < std::numeric_limits<double>::epsilon();
	}
};

/// Edge data for testing, storing ID
class MockEdgeData : public SurgSim::DataStructures::MeshElementData<2>
{
public:
	/// Constructor
	/// \param	id	Unique ID of the edge in its mesh
	MockEdgeData(unsigned int id) : MeshElementData<2>(),
		m_id(id)
	{
	}
	/// Destructor
	virtual ~MockEdgeData()
	{
	}

	/// Gets the edge's unique ID in its mesh.
	unsigned int getId() const
	{
		return m_id;
	}
private:
	/// Edge's unique ID in its mesh
	const unsigned int m_id;

	/// Internal comparison of data of the same type: returns true if equal, false if not equal.
	/// Compares edge ID.
	/// \param	data	Data must be of the same type as that which it is compared against
	virtual bool isEqual(const MeshElementData<2>& data) const
	{
		const MockEdgeData& castData = static_cast<const MockEdgeData&>(data);
		return m_id == castData.m_id;
	}
};

/// Triangle data for testing, storing ID and edge IDs
class MockTriangleData : public SurgSim::DataStructures::MeshElementData<3>
{
public:
	/// Constructor
	/// \param	id	Unique ID of the triangle in its mesh
	/// \param	edges	IDs of the triangle's edges in its mesh
	MockTriangleData(unsigned int id, const std::array<unsigned int, 3>& edges) : MeshElementData<3>(),
		m_id(id),
		m_edges(edges)
	{
	}
	/// Destructor
	virtual ~MockTriangleData()
	{
	}

	/// Gets the triangle's unique ID in its mesh.
	unsigned int getId() const
	{
		return m_id;
	}

	/// Gets the IDs of the triangle's edges in its mesh.
	const std::array<unsigned int, 3>& getEdges() const
	{
		return m_edges;
	}
private:
	/// Edge's unique ID in its mesh
	const unsigned int m_id;
	/// The IDs of the triangle's edges in its mesh, in order: {vertex0->vertex1, vertex1->vertex2, vertex2->vertex3}
	const std::array<unsigned int, 3> m_edges;

	/// Internal comparison of data of the same type: returns true if equal, false if not equal.
	/// Compares triangle ID and edge IDs.
	/// \param	data	Data must be of the same type as that which it is compared against
	virtual bool isEqual(const MeshElementData<3>& data) const
	{
		const MockTriangleData& castData = static_cast<const MockTriangleData&>(data);
		return m_id == castData.m_id && m_edges == castData.m_edges;
	}
};

/// Mesh for testing using MockVertexData
class MockMesh : public SurgSim::DataStructures::Mesh
{
public:
	/// Constructor. Start out with no vertices and 0 updates
	MockMesh() : SurgSim::DataStructures::Mesh(),
		m_numUpdates(0)
	{
	}
	/// Destructor
	virtual ~MockMesh()
	{
	}

	/// Create a new vertex in the mesh
	/// \param position Position of the vertex
	/// \param normal Normal of the vertex
	/// \return Unique ID of vertex in the mesh
	unsigned int createVertex(const SurgSim::Math::Vector3d& position, const SurgSim::Math::Vector3d& normal)
	{
		Vertex vertex(position, std::make_shared<MockVertexData>(getNumVertices(), normal));

		return addVertex(vertex);
	}

	/// Returns the normal of a vertex
	const SurgSim::Math::Vector3d& getVertexNormal(unsigned int id) const
	{
		const std::shared_ptr<MockVertexData>& data = std::static_pointer_cast<MockVertexData>(getVertex(id).data);
		return data->getNormal();
	}

	/// Returns the number of updates performed on the mesh
	int getNumUpdates() const
	{
		return m_numUpdates;
	}

private:
	/// Provides update funcionality, which just increments the number of updates
	virtual void doUpdate()
	{
		++m_numUpdates;
	}

	/// Number of updates performed on the mesh
	int m_numUpdates;
};

#endif SURGSIM_DATA_STRUCTURES_UNITTESTS_MOCK_OBJECTS_H
