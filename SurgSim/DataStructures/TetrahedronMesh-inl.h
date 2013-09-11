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

#ifndef SURGSIM_DATASTRUCTURES_TETRAHEDRONMESH_INL_H
#define SURGSIM_DATASTRUCTURES_TETRAHEDRONMESH_INL_H

namespace SurgSim
{

namespace DataStructures
{

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	TetrahedronMesh()
{
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	~TetrahedronMesh()
{
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
unsigned int
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	addEdge(const EdgeType& edge)
{
	m_edges.push_back(edge);
	return m_edges.size() - 1;
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
unsigned int
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	addTriangle(const TriangleType& triangle)
{
	m_triangles.push_back(triangle);
	return m_triangles.size() - 1;
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
unsigned int
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	addTetrahedron(const TetrahedronType& tetrahedron)
{
	m_tetrahedrons.push_back(tetrahedron);
	return m_tetrahedrons.size() - 1;
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
unsigned int
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	getNumEdges() const
{
	return m_edges.size();
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
unsigned int
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	getNumTriangles() const
{
	return m_triangles.size();
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
unsigned int
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	getNumTetrahedrons() const
{
	return m_tetrahedrons.size();
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
const std::vector<typename TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::EdgeType>&
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	getEdges() const
{
	return m_edges;
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
const std::vector<typename TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::TriangleType>&
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	getTriangles() const
{
	return m_triangles;
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
const std::vector<typename TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::TetrahedronType>&
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	getTetrahedrons() const
{
	return m_tetrahedrons;
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
const typename TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::EdgeType&
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	getEdge(unsigned int id) const
{
	return m_edges[id];
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
const typename TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::TriangleType&
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	getTriangle(unsigned int id) const
{
	return m_triangles[id];
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
const typename TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::TetrahedronType&
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	getTetrahedron(unsigned int id) const
{
	return m_tetrahedrons[id];
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
bool
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	isValid() const
{
	typedef typename TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::EdgeType
		EdgeType;
	typedef typename TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::TriangleType
		TriangleType;
	typedef typename TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::TetrahedronType
		TetrahedronType;

	unsigned int numVertices = Vertices<VertexData>::getNumVertices();

	// Test edges validity
	for (typename std::vector<EdgeType>::const_iterator it = m_edges.begin();
		it != m_edges.end();
		it++)
	{
		for (int vertexId = 0; vertexId < 2; vertexId++)
		{
			if (it->vertices[vertexId] >= numVertices)
			{
				return false;
			}
		}
	}

	// Test triangles validity
	for (typename std::vector<TriangleType>::const_iterator it = m_triangles.begin();
		it != m_triangles.end();
		it++)
	{
		for (int vertexId = 0; vertexId < 3; vertexId++)
		{
			if (it->vertices[vertexId] >= numVertices)
			{
				return false;
			}
		}
	}

	// Test tetrahedrons validity
	for (typename std::vector<TetrahedronType>::const_iterator it = m_tetrahedrons.begin();
		it != m_tetrahedrons.end();
		it++)
	{
		for (int vertexId = 0; vertexId < 4; vertexId++)
		{
			if (it->vertices[vertexId] >= numVertices)
			{
				return false;
			}
		}
	}

	return true;
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
void
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	doClearEdges()
{
	m_edges.clear();
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
void
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	doClearTriangles()
{
	m_triangles.clear();
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
void
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	doClearTetrahedrons()
{
	m_tetrahedrons.clear();
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
bool
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	isEqual(const Vertices<VertexData>& mesh) const
{
	const TetrahedronMesh& tetrahedronMesh = static_cast<const TetrahedronMesh&>(mesh);
	return Vertices<VertexData>::isEqual(mesh) && m_edges == tetrahedronMesh.getEdges() &&
		m_triangles == tetrahedronMesh.getTriangles() && m_tetrahedrons == tetrahedronMesh.getTetrahedrons();
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
void
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::
	doClear()
{
	doClearTetrahedrons();
	doClearTriangles();
	doClearEdges();
	Vertices<VertexData>::doClearVertices();
}

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_TETRAHEDRONMESH_INL_H
