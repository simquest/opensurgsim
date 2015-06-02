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
TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::TetrahedronMesh()
{
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::~TetrahedronMesh()
{
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
size_t TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::addTetrahedron(
		const TetrahedronType& tetrahedron)
{
	m_tetrahedrons.push_back(tetrahedron);
	return m_tetrahedrons.size() - 1;
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
size_t TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>:: getNumTetrahedrons() const
{
	return m_tetrahedrons.size();
}


template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
const std::vector<typename TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::TetrahedronType>&
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::getTetrahedrons() const
{
	return m_tetrahedrons;
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
std::vector<typename TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::TetrahedronType>&
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::getTetrahedrons()
{
	return m_tetrahedrons;
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
const typename TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::TetrahedronType&
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::getTetrahedron(size_t id) const
{
	return m_tetrahedrons[id];
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
typename TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::TetrahedronType&
	TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::getTetrahedron(size_t id)
{
	return m_tetrahedrons[id];
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
bool TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::isValid() const
{
	if (!TriangleMesh<VertexData, EdgeData, TriangleData>::isValid())
	{
		return false;
	}

	typedef typename TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::TetrahedronType
		TetrahedronType;
	size_t numVertices = Vertices<VertexData>::getNumVertices();

	// Test tetrahedrons validity
	for (typename std::vector<TetrahedronType>::const_iterator it = m_tetrahedrons.begin();
		it != m_tetrahedrons.end();
		it++)
	{
		for (size_t vertexId = 0; vertexId < 4; vertexId++)
		{
			if (it->verticesId[vertexId] >= numVertices)
			{
				return false;
			}
		}
	}

	return true;
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
void TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::doClearTetrahedrons()
{
	m_tetrahedrons.clear();
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
bool TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::isEqual(const Vertices<VertexData>& mesh)
	const
{
	const TetrahedronMesh& tetrahedronMesh = static_cast<const TetrahedronMesh&>(mesh);
	return TriangleMesh<VertexData, EdgeData, TriangleData>::isEqual(mesh) &&
		m_tetrahedrons == tetrahedronMesh.getTetrahedrons();
}

template <class VertexData, class EdgeData, class TriangleData, class TetrahedronData>
void TetrahedronMesh<VertexData, EdgeData, TriangleData, TetrahedronData>::doClear()
{
	doClearTetrahedrons();
	doClearTriangles();
	doClearEdges();
	doClearVertices();
}

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_TETRAHEDRONMESH_INL_H
