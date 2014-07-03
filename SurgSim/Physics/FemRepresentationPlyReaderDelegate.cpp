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
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/FemRepresentation.h"
#include "SurgSim/Physics/FemRepresentationPlyReaderDelegate.h"

using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Physics
{

FemRepresentationPlyReaderDelegate::FemRepresentationPlyReaderDelegate(std::shared_ptr<FemRepresentation> fem) :
	m_hasBoundaryConditions(false),
	m_vertexIterator(nullptr),
	m_fem(fem)
{
}

FemRepresentationPlyReaderDelegate::ElementData::ElementData() : indices(nullptr), vertexCount(0)
{
}

bool FemRepresentationPlyReaderDelegate::registerDelegate(PlyReader* reader)
{
	// Vertex processing
	reader->requestElement(
		"vertex",
		std::bind(
		&FemRepresentationPlyReaderDelegate::beginVertices, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&FemRepresentationPlyReaderDelegate::processVertex, this, std::placeholders::_1),
		std::bind(&FemRepresentationPlyReaderDelegate::endVertices, this, std::placeholders::_1));
	reader->requestScalarProperty("vertex", "x", PlyReader::TYPE_DOUBLE, 0 * sizeof(m_vertexData[0]));
	reader->requestScalarProperty("vertex", "y", PlyReader::TYPE_DOUBLE, 1 * sizeof(m_vertexData[0]));
	reader->requestScalarProperty("vertex", "z", PlyReader::TYPE_DOUBLE, 2 * sizeof(m_vertexData[0]));

	// Element Processing
	reader->requestElement(
		getElementName(),
		std::bind(&FemRepresentationPlyReaderDelegate::beginFemElements,
		this,
		std::placeholders::_1,
		std::placeholders::_2),
		std::bind(&FemRepresentationPlyReaderDelegate::processFemElement, this, std::placeholders::_1),
		std::bind(&FemRepresentationPlyReaderDelegate::endFemElements, this, std::placeholders::_1));
	reader->requestListProperty(getElementName(),
		"vertex_indices",
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(ElementData, indices),
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(ElementData, vertexCount));

	// Boundary Condition Processing
	if (m_hasBoundaryConditions)
	{
		reader->requestElement(
			"boundary_condition",
			std::bind(&FemRepresentationPlyReaderDelegate::beginBoundaryConditions,
			this,
			std::placeholders::_1,
			std::placeholders::_2),
			std::bind(&FemRepresentationPlyReaderDelegate::processBoundaryCondition, this, std::placeholders::_1),
			nullptr);
		reader->requestScalarProperty("boundary_condition", "vertex_index", PlyReader::TYPE_UNSIGNED_INT, 0);
	}

	// Material Processing
	reader->requestElement(
		"material",
		std::bind(
		&FemRepresentationPlyReaderDelegate::beginMaterials, this, std::placeholders::_1, std::placeholders::_2),
		nullptr,
		nullptr);
	reader->requestScalarProperty(
		"material", "mass_density", PlyReader::TYPE_DOUBLE, offsetof(MaterialData, massDensity));
	reader->requestScalarProperty(
		"material", "poisson_ratio", PlyReader::TYPE_DOUBLE, offsetof(MaterialData, poissonRatio));
	reader->requestScalarProperty(
		"material", "young_modulus", PlyReader::TYPE_DOUBLE, offsetof(MaterialData, youngModulus));

	reader->setStartParseFileCallback(std::bind(&FemRepresentationPlyReaderDelegate::startParseFile, this));
	reader->setEndParseFileCallback(std::bind(&FemRepresentationPlyReaderDelegate::endParseFile, this));

	return true;
}

bool FemRepresentationPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = true;

	// Shortcut test if one fails ...
	result = result && reader.hasProperty("vertex", "x");
	result = result && reader.hasProperty("vertex", "y");
	result = result && reader.hasProperty("vertex", "z");

	result = result && reader.hasProperty(getElementName(), "vertex_indices");
	result = result && !reader.isScalar(getElementName(), "vertex_indices");

	result = result && reader.hasProperty("material", "mass_density");
	result = result && reader.hasProperty("material", "poisson_ratio");
	result = result && reader.hasProperty("material", "young_modulus");

	m_hasBoundaryConditions = reader.hasProperty("boundary_condition", "vertex_index");

	return result;
}

void FemRepresentationPlyReaderDelegate::startParseFile()
{
	SURGSIM_ASSERT(nullptr != m_fem) << "The FemRepresentation cannot be nullptr.";
	SURGSIM_ASSERT(0 == m_fem->getNumFemElements()) <<
		"The FemRepresentation already contains fem elements, so it cannot be initialized.";
	SURGSIM_ASSERT(nullptr == m_fem->getInitialState()) << "The FemRepresentation already has an initial state";

	m_state = std::make_shared<SurgSim::Math::OdeState>();
}

void FemRepresentationPlyReaderDelegate::endParseFile()
{
	for (size_t i = 0; i < m_fem->getNumFemElements(); ++i)
	{
		m_fem->getFemElement(i)->setMassDensity(m_materialData.massDensity);
		m_fem->getFemElement(i)->setPoissonRatio(m_materialData.poissonRatio);
		m_fem->getFemElement(i)->setYoungModulus(m_materialData.youngModulus);
	}

	m_fem->setInitialState(m_state);
}

void* FemRepresentationPlyReaderDelegate::beginVertices(const std::string& elementName, size_t vertexCount)
{
	m_state->setNumDof(m_fem->getNumDofPerNode(), vertexCount);
	m_vertexIterator = m_state->getPositions().data();

	return m_vertexData.data();
}

void FemRepresentationPlyReaderDelegate::processVertex(const std::string& elementName)
{
	std::copy(std::begin(m_vertexData), std::end(m_vertexData), m_vertexIterator);
	m_vertexIterator += m_fem->getNumDofPerNode();
}

void FemRepresentationPlyReaderDelegate::endVertices(const std::string& elementName)
{
	m_vertexIterator = nullptr;
}

void* FemRepresentationPlyReaderDelegate::beginFemElements(const std::string& elementName, size_t elementCount)
{
	return &m_femData;
}

void FemRepresentationPlyReaderDelegate::endFemElements(const std::string& elementName)
{
	m_femData.indices = nullptr;
}

void* FemRepresentationPlyReaderDelegate::beginMaterials(const std::string& elementName, size_t materialCount)
{
	return &m_materialData;
}

void* FemRepresentationPlyReaderDelegate::beginBoundaryConditions(const std::string& elementName,
																  size_t boundaryConditionCount)
{
	return &m_boundaryConditionData;
}

void FemRepresentationPlyReaderDelegate::processBoundaryCondition(const std::string& elementName)
{
	m_state->addBoundaryCondition(m_boundaryConditionData);
}

} // namespace SurgSim
} // namespace DataStructures
