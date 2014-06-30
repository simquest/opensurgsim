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

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/TriangleMeshPlyReaderDelegate.h"
#include "SurgSim/DataStructures/PlyReader.h"

using SurgSim::DataStructures::EmptyData;

namespace SurgSim
{
namespace DataStructures
{

TriangleMeshPlyReaderDelegate::TriangleMeshPlyReaderDelegate() :
	m_mesh(std::make_shared<MeshType>())
{

}


TriangleMeshPlyReaderDelegate::TriangleMeshPlyReaderDelegate(std::shared_ptr<MeshType> mesh) :
	m_mesh(mesh)
{
	SURGSIM_ASSERT(mesh != nullptr) << "The mesh cannot be null.";
	mesh->clear();
}

bool TriangleMeshPlyReaderDelegate::registerDelegate(PlyReader* reader)
{
	// Vertex processing
	reader->requestElement("vertex",
		std::bind(&TriangleMeshPlyReaderDelegate::beginVertices, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&TriangleMeshPlyReaderDelegate::processVertex, this, std::placeholders::_1),
		std::bind(&TriangleMeshPlyReaderDelegate::endVertices, this, std::placeholders::_1));
	reader->requestScalarProperty("vertex", "x", PlyReader::TYPE_DOUBLE, offsetof(VertexData, x));
	reader->requestScalarProperty("vertex", "y", PlyReader::TYPE_DOUBLE, offsetof(VertexData, y));
	reader->requestScalarProperty("vertex", "z", PlyReader::TYPE_DOUBLE, offsetof(VertexData, z));

	// Face Processing
	reader->requestElement("face",
		std::bind(&TriangleMeshPlyReaderDelegate::beginFaces, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&TriangleMeshPlyReaderDelegate::processFace, this, std::placeholders::_1),
		std::bind(&TriangleMeshPlyReaderDelegate::endFaces, this, std::placeholders::_1));
	reader->requestListProperty("face", "vertex_indices",
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(FaceData, indices),
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(FaceData, edgeCount));

	return true;
}

bool TriangleMeshPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = true;

	// Shortcut test if one fails ...
	result = result && reader.hasProperty("vertex", "x");
	result = result && reader.hasProperty("vertex", "y");
	result = result && reader.hasProperty("vertex", "z");
	result = result && reader.hasProperty("face", "vertex_indices");
	result = result && !reader.isScalar("face", "vertex_indices");

	return result;
}

std::shared_ptr<TriangleMeshBase<EmptyData, EmptyData, EmptyData>> TriangleMeshPlyReaderDelegate::getMesh()
{
	return m_mesh;
}



void* TriangleMeshPlyReaderDelegate::beginVertices(const std::string& elementName, size_t vertexCount)
{
	m_vertexData.overrun = 0l;
	return &m_vertexData;
}

void TriangleMeshPlyReaderDelegate::processVertex(const std::string& elementName)
{
	MeshType::VertexType vertex(SurgSim::Math::Vector3d(m_vertexData.x, m_vertexData.y, m_vertexData.z));
	m_mesh->addVertex(vertex);
}

void TriangleMeshPlyReaderDelegate::endVertices(const std::string& elementName)
{
	SURGSIM_ASSERT(m_vertexData.overrun == 0) <<
		"There was an overrun while reading the vertex structures, it is likely that data " <<
		"has become corrupted.";
}

void* TriangleMeshPlyReaderDelegate::beginFaces(const std::string& elementName, size_t faceCount)
{
	m_faceData.overrun = 0l;
	return &m_faceData;
}

void TriangleMeshPlyReaderDelegate::processFace(const std::string& elementName)
{
	SURGSIM_ASSERT(m_faceData.edgeCount == 3) << "Can only process triangle meshes.";
	std::copy(m_faceData.indices, m_faceData.indices + 3, m_indices.begin());

	TriangleMeshBase<EmptyData, EmptyData, EmptyData>::TriangleType triangle(m_indices);
	m_mesh->addTriangle(triangle);
}

void TriangleMeshPlyReaderDelegate::endFaces(const std::string& elementName)
{
	SURGSIM_ASSERT(m_faceData.overrun == 0) <<
		"There was an overrun while reading the 2DElement structures, it is likely that data " <<
		"has become corrupted.";
}

}
}

