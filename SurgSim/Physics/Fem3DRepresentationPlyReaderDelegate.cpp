// This file is a part of the OpenSurgSim project.
// Copyright 2014, SimQuest Solutions Inc.
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

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Physics/DeformableRepresentationState.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationPlyReaderDelegate.h"
#include "SurgSim/Physics/FemElement3DTetrahedron.h"

using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Physics
{

Fem3DRepresentationPlyReaderDelegate::Fem3DRepresentationPlyReaderDelegate()
	: vertexIterator(nullptr), m_fem(nullptr), m_state(nullptr)
{

}

bool Fem3DRepresentationPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = true;

	// Shortcut test if one fails ...
	result = result && reader.hasProperty("vertex", "x");
	result = result && reader.hasProperty("vertex", "y");
	result = result && reader.hasProperty("vertex", "z");

	result = result && reader.hasProperty("tetrahedron", "vertex_index_0");
	result = result && reader.hasProperty("tetrahedron", "vertex_index_1");
	result = result && reader.hasProperty("tetrahedron", "vertex_index_2");
	result = result && reader.hasProperty("tetrahedron", "vertex_index_3");

	result = result && reader.hasProperty("boundary_condition", "vertex_index");

	return result;
}

bool Fem3DRepresentationPlyReaderDelegate::registerDelegate(PlyReader* reader)
{
	// Vertex processing
	reader->requestElement(
		"vertex",
		std::bind(
			&Fem3DRepresentationPlyReaderDelegate::beginVertices, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&Fem3DRepresentationPlyReaderDelegate::processVertex, this, std::placeholders::_1),
		std::bind(&Fem3DRepresentationPlyReaderDelegate::endVertices, this, std::placeholders::_1));
	reader->requestScalarProperty("vertex", "x", PlyReader::TYPE_DOUBLE, 0 * sizeof(m_vertexData[0]));
	reader->requestScalarProperty("vertex", "y", PlyReader::TYPE_DOUBLE, 1 * sizeof(m_vertexData[0]));
	reader->requestScalarProperty("vertex", "z", PlyReader::TYPE_DOUBLE, 2 * sizeof(m_vertexData[0]));

	// Tetrahedron Processing
	reader->requestElement(
		"tetrahedron",
		std::bind(&Fem3DRepresentationPlyReaderDelegate::beginTetrahedrons,
				  this,
				  std::placeholders::_1,
				  std::placeholders::_2),
		std::bind(&Fem3DRepresentationPlyReaderDelegate::processTetrahedron, this, std::placeholders::_1),
		std::bind(&Fem3DRepresentationPlyReaderDelegate::endTetrahedrons, this, std::placeholders::_1));
	reader->requestScalarProperty(
		"tetrahedron", "vertex_index_0", PlyReader::TYPE_UNSIGNED_INT, 0 * sizeof(m_tetrahedronData[0]));
	reader->requestScalarProperty(
		"tetrahedron", "vertex_index_1", PlyReader::TYPE_UNSIGNED_INT, 1 * sizeof(m_tetrahedronData[0]));
	reader->requestScalarProperty(
		"tetrahedron", "vertex_index_2", PlyReader::TYPE_UNSIGNED_INT, 2 * sizeof(m_tetrahedronData[0]));
	reader->requestScalarProperty(
		"tetrahedron", "vertex_index_3", PlyReader::TYPE_UNSIGNED_INT, 3 * sizeof(m_tetrahedronData[0]));

	// Boundary Condition Processing
	reader->requestElement(
		"boundary_condition",
		std::bind(&Fem3DRepresentationPlyReaderDelegate::beginBoundaryConditions,
				  this,
				  std::placeholders::_1,
				  std::placeholders::_2),
		std::bind(&Fem3DRepresentationPlyReaderDelegate::processBoundaryCondition, this, std::placeholders::_1),
		std::bind(&Fem3DRepresentationPlyReaderDelegate::endBoundaryConditions, this, std::placeholders::_1));
	reader->requestScalarProperty("boundary_condition", "vertex_index", PlyReader::TYPE_UNSIGNED_INT, 0);

	reader->setStartParseFileCallback(std::bind(&Fem3DRepresentationPlyReaderDelegate::startParseFile, this));
	reader->setEndParseFileCallback(std::bind(&Fem3DRepresentationPlyReaderDelegate::endParseFile, this));

	return true;
}

std::shared_ptr<Fem3DRepresentation> Fem3DRepresentationPlyReaderDelegate::getFem()
{
	return m_fem;
}

void Fem3DRepresentationPlyReaderDelegate::startParseFile()
{
	m_fem = std::make_shared<Fem3DRepresentation>("Ply loaded Fem3d");
	m_state = std::make_shared<DeformableRepresentationState>();
}

void Fem3DRepresentationPlyReaderDelegate::endParseFile()
{
	m_fem->setInitialState(m_state);
}

void* Fem3DRepresentationPlyReaderDelegate::beginVertices(const std::string& elementName, size_t vertexCount)
{
	m_state->setNumDof(3, vertexCount);
	vertexIterator = m_state->getPositions().data();

	return m_vertexData.data();
}

void Fem3DRepresentationPlyReaderDelegate::processVertex(const std::string& elementName)
{
	std::copy(std::begin(m_vertexData), std::end(m_vertexData), vertexIterator);
	vertexIterator += 3;
}

void Fem3DRepresentationPlyReaderDelegate::endVertices(const std::string& elementName)
{
	vertexIterator = nullptr;
}

void* Fem3DRepresentationPlyReaderDelegate::beginTetrahedrons(const std::string& elementName, size_t faceCount)
{
	return m_tetrahedronData.data();
}

void Fem3DRepresentationPlyReaderDelegate::processTetrahedron(const std::string& elementName)
{
	m_fem->addFemElement(std::make_shared<FemElement3DTetrahedron>(m_tetrahedronData, *m_state));
}

void Fem3DRepresentationPlyReaderDelegate::endTetrahedrons(const std::string& elementName)
{
}

void* Fem3DRepresentationPlyReaderDelegate::beginBoundaryConditions(const std::string& elementName, size_t faceCount)
{
	return &m_boundaryConditionData;
}

void Fem3DRepresentationPlyReaderDelegate::processBoundaryCondition(const std::string& elementName)
{
	m_state->addBoundaryCondition(m_boundaryConditionData);
}

void Fem3DRepresentationPlyReaderDelegate::endBoundaryConditions(const std::string& elementName)
{
}

} // namespace SurgSim
} // namespace DataStructures
