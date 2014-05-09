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
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationPlyReaderDelegate.h"
#include "SurgSim/Physics/FemElement3DTetrahedron.h"

using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Physics
{

Fem3DRepresentationPlyReaderDelegate::Fem3DRepresentationPlyReaderDelegate(std::shared_ptr<Fem3DRepresentation> fem)
	: vertexIterator(nullptr), m_fem(fem), m_hasBoundaryConditions(false)
{
}

Fem3DRepresentationPlyReaderDelegate::PolyhedronData::PolyhedronData() : indicies(nullptr), vertexCount(0)
{
}

bool Fem3DRepresentationPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = true;

	// Shortcut test if one fails ...
	result = result && reader.hasProperty("vertex", "x");
	result = result && reader.hasProperty("vertex", "y");
	result = result && reader.hasProperty("vertex", "z");

	result = result && reader.hasProperty("polyhedron", "vertex_indices");
	result = result && !reader.isScalar("polyhedron", "vertex_indices");

	result = result && reader.hasProperty("material", "mass_density");
	result = result && reader.hasProperty("material", "poisson_ratio");
	result = result && reader.hasProperty("material", "young_modulus");

	m_hasBoundaryConditions = reader.hasProperty("boundary_condition", "vertex_index");

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

	// Polyhedron Processing
	reader->requestElement(
		"polyhedron",
		std::bind(&Fem3DRepresentationPlyReaderDelegate::beginPolyhedrons,
				  this,
				  std::placeholders::_1,
				  std::placeholders::_2),
		std::bind(&Fem3DRepresentationPlyReaderDelegate::processPolyhedron, this, std::placeholders::_1),
		std::bind(&Fem3DRepresentationPlyReaderDelegate::endPolyhedrons, this, std::placeholders::_1));
	reader->requestListProperty("polyhedron",
								"vertex_indices",
								PlyReader::TYPE_UNSIGNED_INT,
								offsetof(PolyhedronData, indicies),
								PlyReader::TYPE_UNSIGNED_INT,
								offsetof(PolyhedronData, vertexCount));

	reader->requestElement(
		"material",
		std::bind(
			&Fem3DRepresentationPlyReaderDelegate::beginMaterials, this, std::placeholders::_1, std::placeholders::_2),
		nullptr,
		nullptr);
	reader->requestScalarProperty(
		"material", "mass_density", PlyReader::TYPE_DOUBLE, offsetof(MaterialData, massDensity));
	reader->requestScalarProperty(
		"material", "poisson_ratio", PlyReader::TYPE_DOUBLE, offsetof(MaterialData, poissonRatio));
	reader->requestScalarProperty(
		"material", "young_modulus", PlyReader::TYPE_DOUBLE, offsetof(MaterialData, youngModulus));

	// Boundary Condition Processing
	if (m_hasBoundaryConditions)
	{
		reader->requestElement(
			"boundary_condition",
			std::bind(&Fem3DRepresentationPlyReaderDelegate::beginBoundaryConditions,
					  this,
					  std::placeholders::_1,
					  std::placeholders::_2),
			std::bind(&Fem3DRepresentationPlyReaderDelegate::processBoundaryCondition, this, std::placeholders::_1),
			nullptr);
		reader->requestScalarProperty("boundary_condition", "vertex_index", PlyReader::TYPE_UNSIGNED_INT, 0);
	}

	reader->setStartParseFileCallback(std::bind(&Fem3DRepresentationPlyReaderDelegate::startParseFile, this));
	reader->setEndParseFileCallback(std::bind(&Fem3DRepresentationPlyReaderDelegate::endParseFile, this));

	return true;
}

void Fem3DRepresentationPlyReaderDelegate::startParseFile()
{
	SURGSIM_ASSERT(m_fem != nullptr) << "The Representation cannot be nullptr.";
	SURGSIM_ASSERT(m_fem->getNumFemElements() == 0)
		<< "The Representation already contains fem elements, so it cannot be initialized.";
	SURGSIM_ASSERT(m_fem->getInitialState() == nullptr)
		<< "The Representation's initial state must be uninitialized.";

	m_state = std::make_shared<SurgSim::Math::OdeState>();
}

void Fem3DRepresentationPlyReaderDelegate::endParseFile()
{
	for (size_t i = 0; i < m_fem->getNumFemElements(); i++)
	{
		m_fem->getFemElement(i)->setMassDensity(m_materialData.massDensity);
		m_fem->getFemElement(i)->setPoissonRatio(m_materialData.poissonRatio);
		m_fem->getFemElement(i)->setYoungModulus(m_materialData.youngModulus);
	}

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

void* Fem3DRepresentationPlyReaderDelegate::beginPolyhedrons(const std::string& elementName, size_t polyhedronCount)
{
	return &m_polyhedronData;
}

void Fem3DRepresentationPlyReaderDelegate::processPolyhedron(const std::string& elementName)
{
	SURGSIM_ASSERT(m_polyhedronData.vertexCount == 4) << "Cannot process polyhedron with "
			<< m_polyhedronData.vertexCount << " vertices.";

	std::array<unsigned int, 4> polyhedronVertices;
	std::copy(m_polyhedronData.indicies, m_polyhedronData.indicies + 4, polyhedronVertices.begin());
	m_fem->addFemElement(std::make_shared<FemElement3DTetrahedron>(polyhedronVertices));
}

void Fem3DRepresentationPlyReaderDelegate::endPolyhedrons(const std::string& elementName)
{
	m_polyhedronData.indicies = nullptr;
}

void* Fem3DRepresentationPlyReaderDelegate::beginMaterials(const std::string& elementName, size_t materialCount)
{
	return &m_materialData;
}

void* Fem3DRepresentationPlyReaderDelegate::beginBoundaryConditions(const std::string& elementName,
		size_t boundaryConditionCount)
{
	return &m_boundaryConditionData;
}

void Fem3DRepresentationPlyReaderDelegate::processBoundaryCondition(const std::string& elementName)
{
	m_state->addBoundaryCondition(3 * m_boundaryConditionData);
	m_state->addBoundaryCondition(3 * m_boundaryConditionData + 1);
	m_state->addBoundaryCondition(3 * m_boundaryConditionData + 2);
}

} // namespace SurgSim
} // namespace DataStructures
