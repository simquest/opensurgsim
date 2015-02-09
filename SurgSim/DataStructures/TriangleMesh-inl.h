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

#ifndef SURGSIM_DATASTRUCTURES_TRIANGLEMESH_INL_H
#define SURGSIM_DATASTRUCTURES_TRIANGLEMESH_INL_H

#include "SurgSim/Framework/Log.h"


namespace SurgSim
{
namespace DataStructures
{

template <class VertexData, class EdgeData, class TriangleData>
TriangleMesh<VertexData, EdgeData, TriangleData>::TriangleMesh()
{
}

template <class VertexData, class EdgeData, class TriangleData>
TriangleMesh<VertexData, EdgeData, TriangleData>::TriangleMesh(
	const TriangleMesh<VertexData, EdgeData, TriangleData>& other) :
	SurgSim::Framework::Asset(),
	m_edges(other.getEdges()),
	m_triangles(other.getTriangles()),
	m_freeTriangles(other.m_freeTriangles)
{
	for (auto& vertex : other.getVertices())
	{
		addVertex(vertex);
	}
}

template <class VertexData, class EdgeData, class TriangleData>
template <class VertexDataSource, class EdgeDataSource, class TriangleDataSource>
TriangleMesh<VertexData, EdgeData, TriangleData>::TriangleMesh(
	const TriangleMesh<VertexDataSource, EdgeDataSource, TriangleDataSource>& other) :
	SurgSim::Framework::Asset()
{
	for (size_t iVertex = 0; iVertex < other.getNumVertices(); ++iVertex)
	{
		VertexType vertexData(other.getVertexPosition(iVertex));
		addVertex(vertexData);
	}
	for (size_t iEdge = 0; iEdge < other.getNumEdges(); ++iEdge)
	{
		EdgeType edgeData((other.getEdge(iEdge)).verticesId, EdgeData());
		addEdge(edgeData);
	}

	auto& sourceTriangles = other.getTriangles();
	size_t index = 0;
	m_triangles.reserve(sourceTriangles.size());
	for (auto sourceTriangle : sourceTriangles)
	{
		TriangleType triangleData(sourceTriangle.verticesId, TriangleData());
		addTriangle(triangleData);
		if (!sourceTriangle.isValid)
		{
			m_freeTriangles.push_back(index);
		}
		++index;
	}
}

template <class VertexData, class EdgeData, class TriangleData>
TriangleMesh<VertexData, EdgeData, TriangleData>::~TriangleMesh()
{
}

template <class VertexData, class EdgeData, class TriangleData>
std::string TriangleMesh<VertexData, EdgeData, TriangleData>::getClassName() const
{
	return m_className;
}


template <class VertexData, class EdgeData, class TriangleData>
size_t TriangleMesh<VertexData, EdgeData, TriangleData>::addEdge(const EdgeType& edge)
{
	m_edges.push_back(edge);
	return m_edges.size() - 1;
}

template <class VertexData, class EdgeData, class TriangleData>
size_t TriangleMesh<VertexData, EdgeData, TriangleData>::addTriangle(const TriangleType& triangle)
{
	size_t result;

	SURGSIM_ASSERT(triangle.isValid) << "Cannot insert invalid triangle into mesh.";

	if (m_freeTriangles.empty())
	{
		m_triangles.push_back(triangle);
		result = m_triangles.size() - 1;
	}
	else
	{
		result = m_freeTriangles.back();
		m_freeTriangles.pop_back();
		m_triangles[result] = triangle;
	}

	return result;
}

template <class VertexData, class EdgeData, class TriangleData>
size_t TriangleMesh<VertexData, EdgeData, TriangleData>::getNumEdges() const
{
	return m_edges.size();
}

template <class VertexData, class EdgeData, class TriangleData>
size_t TriangleMesh<VertexData, EdgeData, TriangleData>::getNumTriangles() const
{
	return m_triangles.size() - m_freeTriangles.size();
}

template <class VertexData, class EdgeData, class TriangleData>
const std::vector<typename TriangleMesh<VertexData, EdgeData, TriangleData>::EdgeType>&
TriangleMesh<VertexData, EdgeData, TriangleData>::getEdges() const
{
	return m_edges;
}

template <class VertexData, class EdgeData, class TriangleData>
std::vector<typename TriangleMesh<VertexData, EdgeData, TriangleData>::EdgeType>&
TriangleMesh<VertexData, EdgeData, TriangleData>::getEdges()
{
	return m_edges;
}

template <class VertexData, class EdgeData, class TriangleData>
const std::vector<typename TriangleMesh<VertexData, EdgeData, TriangleData>::TriangleType>&
TriangleMesh<VertexData, EdgeData, TriangleData>::getTriangles() const
{
	return m_triangles;
}

template <class VertexData, class EdgeData, class TriangleData>
std::vector<typename TriangleMesh<VertexData, EdgeData, TriangleData>::TriangleType>&
TriangleMesh<VertexData, EdgeData, TriangleData>::getTriangles()
{
	return m_triangles;
}

template <class VertexData, class EdgeData, class TriangleData>
const typename TriangleMesh<VertexData, EdgeData, TriangleData>::EdgeType&
TriangleMesh<VertexData, EdgeData, TriangleData>::getEdge(size_t id) const
{
	return m_edges[id];
}

template <class VertexData, class EdgeData, class TriangleData>
typename TriangleMesh<VertexData, EdgeData, TriangleData>::EdgeType&
TriangleMesh<VertexData, EdgeData, TriangleData>::getEdge(size_t id)
{
	return m_edges[id];
}

template <class VertexData, class EdgeData, class TriangleData>
const typename TriangleMesh<VertexData, EdgeData, TriangleData>::TriangleType&
TriangleMesh<VertexData, EdgeData, TriangleData>::getTriangle(size_t id) const
{
	auto const& triangle = m_triangles[id];
	SURGSIM_ASSERT(triangle.isValid == true) << "Attempted to access invalid or deleted triangle.";
	return triangle;
}

template <class VertexData, class EdgeData, class TriangleData>
typename TriangleMesh<VertexData, EdgeData, TriangleData>::TriangleType&
TriangleMesh<VertexData, EdgeData, TriangleData>::getTriangle(size_t id)
{
	auto& triangle = m_triangles[id];
	SURGSIM_ASSERT(triangle.isValid == true) << "Attempted to access invalid or deleted triangle.";
	return triangle;
}

template <class VertexData, class EdgeData, class TriangleData>
void TriangleMesh<VertexData, EdgeData, TriangleData>::removeTriangle(size_t id)
{
	auto& triangle = m_triangles[id];
	if (triangle.isValid)
	{
		triangle.isValid = false;
		m_freeTriangles.push_back(id);
	}
}

template <class VertexData, class EdgeData, class TriangleData>
std::array<SurgSim::Math::Vector3d, 3>
TriangleMesh<VertexData, EdgeData, TriangleData>::getTrianglePositions(size_t id) const
{
	auto& ids = getTriangle(id).verticesId;
	std::array<SurgSim::Math::Vector3d, 3> result
	= {{
			Vertices<VertexData>::getVertex(ids[0]).position,
			Vertices<VertexData>::getVertex(ids[1]).position,
			Vertices<VertexData>::getVertex(ids[2]).position
		}
	};

	return result;
}

template <class VertexData, class EdgeData, class TriangleData>
bool TriangleMesh<VertexData, EdgeData, TriangleData>::isValid() const
{
	typedef typename TriangleMesh<VertexData, EdgeData, TriangleData>::EdgeType EdgeType;
	typedef typename TriangleMesh<VertexData, EdgeData, TriangleData>::TriangleType TriangleType;

	size_t numVertices = Vertices<VertexData>::getNumVertices();

	// Test edges validity
	for (typename std::vector<EdgeType>::const_iterator it = m_edges.begin(); it != m_edges.end(); ++it)
	{
		for (int vertexId = 0; vertexId < 2; vertexId++)
		{
			if (it->verticesId[vertexId] >= numVertices)
			{
				return false;
			}
		}
	}

	// Test triangles validity
	for (typename std::vector<TriangleType>::const_iterator it = m_triangles.begin(); it != m_triangles.end(); ++it)
	{
		for (int vertexId = 0; vertexId < 3; vertexId++)
		{
			if (it->verticesId[vertexId] >= numVertices)
			{
				return false;
			}
		}
	}

	return true;
}


template <class VertexData, class EdgeData, class TriangleData>
void TriangleMesh<VertexData, EdgeData, TriangleData>::doClearEdges()
{
	m_edges.clear();
}

template <class VertexData, class EdgeData, class TriangleData>
void TriangleMesh<VertexData, EdgeData, TriangleData>::doClearTriangles()
{
	m_triangles.clear();
	m_freeTriangles.clear();
}

template <class VertexData, class EdgeData, class TriangleData>
bool TriangleMesh<VertexData, EdgeData, TriangleData>::isEqual(const Vertices<VertexData>& mesh) const
{
	const TriangleMesh& triangleMesh = static_cast<const TriangleMesh&>(mesh);
	return Vertices<VertexData>::isEqual(triangleMesh) && m_edges == triangleMesh.getEdges() &&
		   m_triangles == triangleMesh.getTriangles();
}

template <class VertexData, class EdgeData, class TriangleData>
bool TriangleMesh<VertexData, EdgeData, TriangleData>::doLoad(const std::string& fileName)
{
	PlyReader reader(fileName);
	if (! reader.isValid())
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
			<< "'" << fileName << "' is an invalid .ply file.";
		return false;
	}

	typedef TriangleMesh<VertexData, EdgeData, TriangleData> MeshType;
	auto delegate = std::make_shared<TriangleMeshPlyReaderDelegate<MeshType>>(this->shared_from_this());
	if (! reader.parseWithDelegate(delegate))
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
			<< "The input file '" << fileName << "' does not have the property required by triangle mesh.";
		return false;
	}

	return true;
}

template <class VertexData, class EdgeData, class TriangleData>
void TriangleMesh<VertexData, EdgeData, TriangleData>::doClear()
{
	doClearTriangles();
	doClearEdges();
	this->doClearVertices();
}



};  // namespace DataStructures
};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_TRIANGLEMESH_INL_H
