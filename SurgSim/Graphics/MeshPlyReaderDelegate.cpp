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

#include "SurgSim/Graphics/MeshPlyReaderDelegate.h"

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/PlyReader.h"

using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Graphics
{

MeshPlyReaderDelegate::MeshPlyReaderDelegate() :
	m_mesh(std::make_shared<MeshType>()),
	m_hasTextureCoordinates(false)
{

}


MeshPlyReaderDelegate::MeshPlyReaderDelegate(std::shared_ptr<MeshType> mesh) :
	m_mesh(mesh),
	m_hasTextureCoordinates(false)
{
	SURGSIM_ASSERT(mesh != nullptr) << "The mesh cannot be null.";
	mesh->clear();
}

bool MeshPlyReaderDelegate::registerDelegate(SurgSim::DataStructures::PlyReader* reader)
{
	// Vertex processing
	reader->requestElement("vertex", std::bind(&MeshPlyReaderDelegate::beginVertices, this,
						   std::placeholders::_1, std::placeholders::_2),
						   std::bind(&MeshPlyReaderDelegate::processVertex, this, std::placeholders::_1),
						   std::bind(&MeshPlyReaderDelegate::endVertices, this, std::placeholders::_1));
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

	// Face Processing
	reader->requestElement("face", std::bind(&MeshPlyReaderDelegate::beginFaces,
						   this, std::placeholders::_1, std::placeholders::_2),
						   std::bind(&MeshPlyReaderDelegate::processFace, this, std::placeholders::_1),
						   std::bind(&MeshPlyReaderDelegate::endFaces, this, std::placeholders::_1));
	reader->requestListProperty("face", "vertex_indices",
								PlyReader::TYPE_UNSIGNED_INT,
								offsetof(FaceData, indices),
								PlyReader::TYPE_UNSIGNED_INT,
								offsetof(FaceData, edgeCount));

	return true;
}

bool MeshPlyReaderDelegate::fileIsAcceptable(const SurgSim::DataStructures::PlyReader& reader)
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

std::shared_ptr<MeshPlyReaderDelegate::MeshType> MeshPlyReaderDelegate::getMesh()
{
	return m_mesh;
}



void* MeshPlyReaderDelegate::beginVertices(const std::string& elementName, size_t vertexCount)
{
	m_vertexData.overrun1 = 0l;
	m_vertexData.overrun2 = 0l;
	return &m_vertexData;
}

void MeshPlyReaderDelegate::processVertex(const std::string& elementName)
{
	MeshType::VertexType vertex(SurgSim::Math::Vector3d(m_vertexData.x, m_vertexData.y, m_vertexData.z));

	if (m_hasTextureCoordinates)
	{
		vertex.data.texture.setValue(SurgSim::Math::Vector2d(m_vertexData.s, m_vertexData.t));
	}

	m_mesh->addVertex(vertex);
}

void MeshPlyReaderDelegate::endVertices(const std::string& elementName)
{
	SURGSIM_ASSERT(m_vertexData.overrun1 == 0 && m_vertexData.overrun2 == 0) <<
			"There was an overrun while reading the vertex structures, it is likely that data " <<
			"has become corrupted.";
}

void* MeshPlyReaderDelegate::beginFaces(const std::string& elementName, size_t faceCount)
{
	m_faceData.overrun = 0l;
	return &m_faceData;
}

void MeshPlyReaderDelegate::processFace(const std::string& elementName)
{
	SURGSIM_ASSERT(m_faceData.edgeCount == 3) << "Can only process triangle meshes.";
	std::copy(m_faceData.indices, m_faceData.indices + 3, m_indices.begin());

	MeshType::TriangleType triangle(m_indices);
	m_mesh->addTriangle(triangle);
}

void MeshPlyReaderDelegate::endFaces(const std::string& elementName)
{
	SURGSIM_ASSERT(m_faceData.overrun == 0)
			<< "There was an overrun while reading the face structures, it is likely that data "
			<< "has become corrupted.";
}

}
}

