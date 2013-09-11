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

namespace SurgSim
{

namespace DataStructures
{

template <class VertexData, class EdgeData, class TriangleData>
TriangleMesh<VertexData, EdgeData, TriangleData>::TriangleMesh()
{
}

template <class VertexData, class EdgeData, class TriangleData>
TriangleMesh<VertexData, EdgeData, TriangleData>::~TriangleMesh()
{
}

template <class VertexData, class EdgeData, class TriangleData>
unsigned int TriangleMesh<VertexData, EdgeData, TriangleData>::addEdge(const EdgeType& edge)
{
	m_edges.push_back(edge);
	return m_edges.size() - 1;
}

template <class VertexData, class EdgeData, class TriangleData>
unsigned int TriangleMesh<VertexData, EdgeData, TriangleData>::addTriangle(const TriangleType& triangle)
{
	m_triangles.push_back(triangle);
	return m_triangles.size() - 1;
}

template <class VertexData, class EdgeData, class TriangleData>
unsigned int TriangleMesh<VertexData, EdgeData, TriangleData>::getNumEdges() const
{
	return m_edges.size();
}

template <class VertexData, class EdgeData, class TriangleData>
unsigned int TriangleMesh<VertexData, EdgeData, TriangleData>::getNumTriangles() const
{
	return m_triangles.size();
}

template <class VertexData, class EdgeData, class TriangleData>
const std::vector<typename TriangleMesh<VertexData, EdgeData, TriangleData>::EdgeType>&
	TriangleMesh<VertexData, EdgeData, TriangleData>::getEdges() const
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
const typename TriangleMesh<VertexData, EdgeData, TriangleData>::EdgeType&
	TriangleMesh<VertexData, EdgeData, TriangleData>::getEdge(unsigned int id) const
{
	return m_edges[id];
}

template <class VertexData, class EdgeData, class TriangleData>
const typename TriangleMesh<VertexData, EdgeData, TriangleData>::TriangleType&
	TriangleMesh<VertexData, EdgeData, TriangleData>::getTriangle(unsigned int id) const
{
	return m_triangles[id];
}

template <class VertexData, class EdgeData, class TriangleData>
bool TriangleMesh<VertexData, EdgeData, TriangleData>::isValid() const
{
	typedef typename TriangleMesh<VertexData, EdgeData, TriangleData>::EdgeType EdgeType;
	typedef typename TriangleMesh<VertexData, EdgeData, TriangleData>::TriangleType TriangleType;

	unsigned int numVertices = Vertices<VertexData>::getNumVertices();

	// Test edges validity
	for (typename std::vector<EdgeType>::const_iterator it = m_edges.begin(); it != m_edges.end(); it++)
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
	for (typename std::vector<TriangleType>::const_iterator it = m_triangles.begin(); it != m_triangles.end(); it++)
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
}

template <class VertexData, class EdgeData, class TriangleData>
bool TriangleMesh<VertexData, EdgeData, TriangleData>::isEqual(const Vertices<VertexData>& mesh) const
{
	const TriangleMesh& triangleMesh = static_cast<const TriangleMesh&>(mesh);
	return Vertices<VertexData>::isEqual(triangleMesh) && m_edges == triangleMesh.getEdges() &&
		m_triangles == triangleMesh.getTriangles();
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
