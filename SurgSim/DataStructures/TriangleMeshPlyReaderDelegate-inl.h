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

#ifndef SURGSIM_DATASTRUCTURES_TRIANGLEMESHPLYREADERDELEGATE_INL_H
#define SURGSIM_DATASTRUCTURES_TRIANGLEMESHPLYREADERDELEGATE_INL_H

#include <cstddef>

#include "SurgSim/Math/Vector.h"

template <class M>
SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<M>::TriangleMeshPlyReaderDelegate() :
	m_mesh(std::make_shared<M>())
{

}

template <class M>
SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<M>::TriangleMeshPlyReaderDelegate(std::shared_ptr<M> mesh) :
	m_mesh(mesh)
{
	SURGSIM_ASSERT(mesh != nullptr) << "The mesh cannot be null.";
	mesh->clear();
}

template <class M>
std::shared_ptr<M> SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<M>::getMesh()
{
	return m_mesh;
}

template <class M>
bool SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<M>::registerDelegate(PlyReader* reader)
{
	// Vertex processing
	reader->requestElement("vertex",
						   std::bind(&TriangleMeshPlyReaderDelegate::beginVertices, this,
									 std::placeholders::_1, std::placeholders::_2),
						   std::bind(&TriangleMeshPlyReaderDelegate::processVertex, this, std::placeholders::_1),
						   std::bind(&TriangleMeshPlyReaderDelegate::endVertices, this, std::placeholders::_1));
	reader->requestScalarProperty("vertex", "x", PlyReader::TYPE_DOUBLE, offsetof(VertexData, x));
	reader->requestScalarProperty("vertex", "y", PlyReader::TYPE_DOUBLE, offsetof(VertexData, y));
	reader->requestScalarProperty("vertex", "z", PlyReader::TYPE_DOUBLE, offsetof(VertexData, z));

	// Normal processing
	m_hasTextureCoordinates = reader->hasProperty("vertex", "s") && reader->hasProperty("vertex", "t");

	if (m_hasTextureCoordinates)
	{
		reader->requestScalarProperty("vertex", "s", PlyReader::TYPE_DOUBLE, offsetof(VertexData, s));
		reader->requestScalarProperty("vertex", "t", PlyReader::TYPE_DOUBLE, offsetof(VertexData, t));
	}

	m_hasFaces = reader->hasProperty("face", "vertex_indices") && !reader->isScalar("face", "vertex_indices");

	if (m_hasFaces)
	{
		// Face Processing
		reader->requestElement("face",
							   std::bind(&TriangleMeshPlyReaderDelegate::beginFaces, this,
										 std::placeholders::_1, std::placeholders::_2),
							   std::bind(&TriangleMeshPlyReaderDelegate::processFace, this, std::placeholders::_1),
							   std::bind(&TriangleMeshPlyReaderDelegate::endFaces, this, std::placeholders::_1));
		reader->requestListProperty("face", "vertex_indices",
									PlyReader::TYPE_UNSIGNED_INT,
									offsetof(ListData, indices),
									PlyReader::TYPE_UNSIGNED_INT,
									offsetof(ListData, count));
	}

	bool hasPhysicsEdges = reader->hasProperty("1d_element", "vertex_indices") &&
						   ! reader->isScalar("1d_element", "vertex_indices");

	if (hasPhysicsEdges)
	{
		// Edge Processing
		reader->requestElement("1d_element",
							   std::bind(&TriangleMeshPlyReaderDelegate::beginEdges, this,
										 std::placeholders::_1, std::placeholders::_2),
							   std::bind(&TriangleMeshPlyReaderDelegate::processEdge, this, std::placeholders::_1),
							   std::bind(&TriangleMeshPlyReaderDelegate::endEdges, this, std::placeholders::_1));
		reader->requestListProperty("1d_element", "vertex_indices",
									PlyReader::TYPE_UNSIGNED_INT,
									offsetof(ListData, indices),
									PlyReader::TYPE_UNSIGNED_INT,
									offsetof(ListData, count));
	}

	reader->setEndParseFileCallback(std::bind(&TriangleMeshPlyReaderDelegate::endFile, this));

	return true;
}

template <class M>
bool SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<M>::fileIsAcceptable(const PlyReader& reader)
{
	bool result = true;

	// Shortcut test if one fails ...
	result = result && reader.hasProperty("vertex", "x");
	result = result && reader.hasProperty("vertex", "y");
	result = result && reader.hasProperty("vertex", "z");

	return result;
}

template <class M>
void* SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<M>::beginVertices(
	const std::string& elementName,
	size_t vertexCount)
{
	m_vertexData.overrun1 = 0l;
	m_vertexData.overrun2 = 0l;
	return &m_vertexData;
}

template <class M>
void SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<M>::processVertex(const std::string& elementName)
{
	typename M::VertexType vertex(SurgSim::Math::Vector3d(m_vertexData.x, m_vertexData.y, m_vertexData.z));
	m_mesh->addVertex(vertex);
}

template <class M>
void SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<M>::endVertices(const std::string& elementName)
{
	SURGSIM_ASSERT(m_vertexData.overrun1 == 0l && m_vertexData.overrun2 == 0l) <<
			"There was an overrun while reading the vertex structures, it is likely that data " <<
			"has become corrupted.";
}

template <class M>
void* SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<M>::beginFaces(
	const std::string& elementName,
	size_t faceCount)
{
	m_listData.overrun = 0l;
	return &m_listData;
}

template <class M>
void SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<M>::processFace(const std::string& elementName)
{
	SURGSIM_ASSERT(m_listData.count == 3) << "Can only process triangle meshes.";
	std::copy(m_listData.indices, m_listData.indices + 3, m_indices.begin());

	typename M::TriangleType triangle(m_indices);
	m_mesh->addTriangle(triangle);
}

template <class M>
void SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<M>::endFaces(const std::string& elementName)
{
	SURGSIM_ASSERT(m_listData.overrun == 0l)
			<< "There was an overrun while reading the face structures, it is likely that data "
			<< "has become corrupted.";
}

template <class M>
void SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<M>::endFile()
{
	m_mesh->update();
}

template <class M>
bool SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<M>::hasTextureCoordinates()
{
	return m_hasTextureCoordinates;
}

template <class M>
void* SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<M>::beginEdges(const std::string& elementName,
		size_t edgeCount)
{
	m_listData.overrun = 0l;
	return &m_listData;
}


template <class M>
void SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<M>::processEdge(const std::string& elementName)
{
	SURGSIM_ASSERT(m_listData.count == 2) << "Edges have to have 2 points.";
	std::copy(m_listData.indices, m_listData.indices + 2, m_edges.begin());

	typename M::EdgeType edge(m_edges);
	m_mesh->addEdge(edge);
}


template <class M>
void SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<M>::endEdges(const std::string& elementName)
{
	SURGSIM_ASSERT(m_listData.overrun == 0l)
			<< "There was an overrun while reading the face structures, it is likely that data "
			<< "has become corrupted.";
}


#endif
