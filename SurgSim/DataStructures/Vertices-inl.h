// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_DATASTRUCTURES_VERTICES_INL_H
#define SURGSIM_DATASTRUCTURES_VERTICES_INL_H

#include <typeinfo>

#include "SurgSim/Framework/Assert.h"


namespace SurgSim
{
namespace DataStructures
{

template <class VertexData>
Vertices<VertexData>::Vertices()
{
}

template <class VertexData>
template <class V>
Vertices<VertexData>::Vertices(const Vertices<V>& other)
{
	m_vertices.reserve(other.getVertices().size());
	for (auto& otherVertex : other.getVertices())
	{
		addVertex(VertexType(otherVertex));
	}
}

template <class VertexData>
template <class V>
Vertices<VertexData>& Vertices<VertexData>::operator=(const Vertices<V>& other)
{
	auto& otherVertices = other.getVertices();

	if (otherVertices.size() < m_vertices.size())
	{
		m_vertices.resize(otherVertices.size());
	}
	else
	{
		m_vertices.reserve(otherVertices.size());
	}

	auto vertex = m_vertices.begin();
	auto otherVertex = otherVertices.begin();
	for (; vertex != m_vertices.end(); ++vertex, ++otherVertex)
	{
		*vertex = *otherVertex;
	}
	for (; otherVertex != otherVertices.end(); ++otherVertex)
	{
		addVertex(VertexType(*otherVertex));
	}

	return *this;
}

template <class VertexData>
Vertices<VertexData>::~Vertices()
{
}

template <class VertexData>
void Vertices<VertexData>::clear()
{
	doClear();
}

template <class VertexData>
bool Vertices<VertexData>::update()
{
	m_updateCount++;
	return doUpdate();
}

size_t Vertices<VertexData>::getUpdateCount() const
{
	return m_updateCount;
}

template <class VertexData>
size_t Vertices<VertexData>::addVertex(const VertexType& vertex)
{
	m_vertices.push_back(vertex);
	return m_vertices.size() - 1;
}

template <class VertexData>
size_t Vertices<VertexData>::getNumVertices() const
{
	return m_vertices.size();
}

template <class VertexData>
const typename Vertices<VertexData>::VertexType& Vertices<VertexData>::getVertex(size_t id) const
{
	return m_vertices[id];
}

template <class VertexData>
typename Vertices<VertexData>::VertexType& Vertices<VertexData>::getVertex(size_t id)
{
	return m_vertices[id];
}

template <class VertexData>
const std::vector<typename Vertices<VertexData>::VertexType>& Vertices<VertexData>::getVertices() const
{
	return m_vertices;
}

template <class VertexData>
std::vector<typename Vertices<VertexData>::VertexType>& Vertices<VertexData>::getVertices()
{
	return m_vertices;
}

template <class VertexData>
void Vertices<VertexData>::setVertexPosition(size_t id, const SurgSim::Math::Vector3d& position)
{
	m_vertices[id].position = position;
}

template <class VertexData>
const SurgSim::Math::Vector3d& Vertices<VertexData>::getVertexPosition(size_t id) const
{
	return m_vertices[id].position;
}

template <class VertexData>
void Vertices<VertexData>::setVertexPositions(const std::vector<SurgSim::Math::Vector3d>& positions, bool doUpdate)
{
	SURGSIM_ASSERT(m_vertices.size() == positions.size()) << "Number of positions must match number of vertices.";

	for (size_t i = 0; i < m_vertices.size(); ++i)
	{
		m_vertices[i].position = positions[i];
	}

	if (doUpdate)
	{
		update();
	}
}

template <class VertexData>
void Vertices<VertexData>::transform(const Math::RigidTransform3d& pose)
{
	for (auto& vertex : m_vertices)
	{
		vertex.position = pose * vertex.position;
	}
}

template <class VertexData>
bool Vertices<VertexData>::operator==(const Vertices& mesh) const
{
	return (typeid(*this) == typeid(mesh)) && isEqual(mesh);
}

template <class VertexData>
bool Vertices<VertexData>::operator!=(const Vertices& mesh) const
{
	return (typeid(*this) != typeid(mesh)) || ! isEqual(mesh);
}

template <class VertexData>
void Vertices<VertexData>::doClearVertices()
{
	m_vertices.clear();
}

template <class VertexData>
bool Vertices<VertexData>::isEqual(const Vertices& mesh) const
{
	return m_vertices == mesh.m_vertices;
}

template <class VertexData>
void Vertices<VertexData>::doClear()
{
	doClearVertices();
}

template <class VertexData>
bool Vertices<VertexData>::doUpdate()
{
	return true;
}

};
};

#endif //SURGSIM_DATASTRUCTURES_VERTICES_INL_H

