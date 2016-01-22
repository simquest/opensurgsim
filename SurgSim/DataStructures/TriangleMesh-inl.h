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
	Vertices<VertexData>::Vertices(other),
	SurgSim::Framework::Asset(),
	m_edges(other.getEdges()),
	m_triangles(other.getTriangles()),
	m_freeTriangles(other.m_freeTriangles)
{
}

template <class VertexData, class EdgeData, class TriangleData>
template <class V, class E, class T>
TriangleMesh<VertexData, EdgeData, TriangleData>::TriangleMesh(const TriangleMesh<V, E, T>& other) :
	Vertices<VertexData>::Vertices(other),
	SurgSim::Framework::Asset()
{
	m_edges.reserve(other.getEdges().size());
	for (auto& edge : other.getEdges())
	{
		addEdge(EdgeType(edge));
	}

	size_t index = 0;
	m_triangles.reserve(other.getTriangles().size());
	for (auto& triangle : other.getTriangles())
	{
		addTriangle(TriangleType(triangle));
		if (!triangle.isValid)
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
std::array<SurgSim::Math::Vector3d, 2>
TriangleMesh<VertexData, EdgeData, TriangleData>::getEdgePositions(size_t id) const
{
	auto& ids = getEdge(id).verticesId;
	std::array<SurgSim::Math::Vector3d, 2> result =
	{{
		Vertices<VertexData>::getVertex(ids[0]).position,
		Vertices<VertexData>::getVertex(ids[1]).position
	}};

	return result;
}

template <class VertexData, class EdgeData, class TriangleData>
const typename TriangleMesh<VertexData, EdgeData, TriangleData>::TriangleType&
TriangleMesh<VertexData, EdgeData, TriangleData>::getTriangle(size_t id) const
{
	auto const& triangle = m_triangles[id];
	SURGSIM_ASSERT(triangle.isValid)
			<< "Attempted to access invalid or deleted triangle " << id << " have " << getNumTriangles();
	return triangle;
}

template <class VertexData, class EdgeData, class TriangleData>
typename TriangleMesh<VertexData, EdgeData, TriangleData>::TriangleType&
TriangleMesh<VertexData, EdgeData, TriangleData>::getTriangle(size_t id)
{
	auto& triangle = m_triangles[id];
	SURGSIM_ASSERT(triangle.isValid) << "Attempted to access invalid or deleted triangle.";
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
	std::array<SurgSim::Math::Vector3d, 3> result =
	{{
		Vertices<VertexData>::getVertex(ids[0]).position,
		Vertices<VertexData>::getVertex(ids[1]).position,
		Vertices<VertexData>::getVertex(ids[2]).position
	}};

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
	doClearVertices();
}

template <class VertexData, class EdgeData, class TriangleData>
TriangleMesh<VertexData, EdgeData, TriangleData>::TriangleMesh(TriangleMesh&& other) :
	Vertices<VertexData>::Vertices(std::move(other))
{
	doClearTriangles();
	doClearEdges();
	std::swap(m_triangles, other.m_triangles);
	std::swap(m_edges, other.m_edges);
	std::swap(m_freeTriangles, other.m_freeTriangles);
}

template <class VertexData, class EdgeData, class TriangleData>
TriangleMesh<VertexData, EdgeData, TriangleData>& TriangleMesh<VertexData, EdgeData, TriangleData>::operator=(const
		TriangleMesh<VertexData, EdgeData, TriangleData>& other)
{
	Vertices<VertexData>::operator=(other);
	m_triangles = other.m_triangles;
	m_edges = other.m_edges;
	m_freeTriangles = other.m_freeTriangles;
	return *this;
}

template <class VertexData, class EdgeData, class TriangleData>
TriangleMesh<VertexData, EdgeData, TriangleData>& TriangleMesh<VertexData, EdgeData, TriangleData>::operator=
(TriangleMesh<VertexData, EdgeData, TriangleData>&& other)
{
	Vertices<VertexData>::operator=(std::move(other));
	doClearTriangles();
	doClearEdges();
	std::swap(m_triangles, other.m_triangles);
	std::swap(m_edges, other.m_edges);
	std::swap(m_freeTriangles, other.m_freeTriangles);
	return *this;
}

template <class VertexData, class EdgeData, class TriangleData>
void SurgSim::DataStructures::TriangleMesh<VertexData, EdgeData, TriangleData>::save(const std::string& fileName)
{
	std::fstream out(fileName, std::ios::out);

	if (out.is_open())
	{
		out << "ply" << std::endl;
		out << "format ascii 1.0" << std::endl;
		out << "comment Created by OpenSurgSim, www.opensurgsim.org" << std::endl;
		out << "element vertex " << getNumVertices() << std::endl;
		out << "property float x\nproperty float y\nproperty float z" << std::endl;
		out << "element face " << getNumTriangles() << std::endl;
		out << "property list uchar uint vertex_indices" << std::endl;
		out << "end_header" << std::endl;

		for (const auto& vertex : getVertices())
		{
			out << vertex.position[0] << " " << vertex.position[1] << " " << vertex.position[2] << std::endl;
		}

		for (const auto& tri : getTriangles())
		{
			out << "3 " << tri.verticesId[0] << " " << tri.verticesId[1] << " " << tri.verticesId[2] << std::endl;
		}

		if (out.bad())
		{
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) << __FUNCTION__
					<< "There was a problem writing " << fileName;
		}

		out.close();
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) << __FUNCTION__
				<< "Could not open " << fileName << " for writing.";
	}

}



};  // namespace DataStructures
};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_TRIANGLEMESH_INL_H
