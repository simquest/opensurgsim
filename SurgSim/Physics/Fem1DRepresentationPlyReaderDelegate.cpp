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

#include <array>

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/Fem1DRepresentationPlyReaderDelegate.h"

namespace SurgSim
{
namespace Physics
{
using SurgSim::DataStructures::PlyReader;

Fem1DRepresentationPlyReaderDelegate::Fem1DRepresentationPlyReaderDelegate(std::shared_ptr<Fem1DRepresentation> fem)
	: FemRepresentationPlyReaderDelegate(fem)
{
}

bool Fem1DRepresentationPlyReaderDelegate::registerDelegate(PlyReader* reader)
{
	// Vertex processing
	reader->requestElement(
		"vertex",
		std::bind(
			&Fem1DRepresentationPlyReaderDelegate::beginVertices, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&Fem1DRepresentationPlyReaderDelegate::processVertex, this, std::placeholders::_1),
		std::bind(&Fem1DRepresentationPlyReaderDelegate::endVertices, this, std::placeholders::_1));
	reader->requestScalarProperty("vertex", "x", PlyReader::TYPE_DOUBLE, 0 * sizeof(m_vertexData[0]));
	reader->requestScalarProperty("vertex", "y", PlyReader::TYPE_DOUBLE, 1 * sizeof(m_vertexData[0]));
	reader->requestScalarProperty("vertex", "z", PlyReader::TYPE_DOUBLE, 2 * sizeof(m_vertexData[0]));

	// Polyhedron Processing
	reader->requestElement(
		"1DElement",
		std::bind(&Fem1DRepresentationPlyReaderDelegate::beginFemElements,
		this,
		std::placeholders::_1,
		std::placeholders::_2),
		std::bind(&Fem1DRepresentationPlyReaderDelegate::processFemElement, this, std::placeholders::_1),
		std::bind(&Fem1DRepresentationPlyReaderDelegate::endFemElements, this, std::placeholders::_1));
	reader->requestListProperty("1DElement",
		"vertex_indices",
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(ElementData, indicies),
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(ElementData, vertexCount));

	// Boundary Condition Processing
	if (m_hasBoundaryConditions)
	{
		reader->requestElement(
			"boundary_condition",
			std::bind(&Fem1DRepresentationPlyReaderDelegate::beginBoundaryConditions,
			this,
			std::placeholders::_1,
			std::placeholders::_2),
			std::bind(&Fem1DRepresentationPlyReaderDelegate::processBoundaryCondition, this, std::placeholders::_1),
			nullptr);
		reader->requestScalarProperty("boundary_condition", "vertex_index", PlyReader::TYPE_UNSIGNED_INT, 0);
	}

	reader->requestElement(
		"radius",
		std::bind(
		&Fem1DRepresentationPlyReaderDelegate::beginRadius, this, std::placeholders::_1, std::placeholders::_2),
		nullptr,
		nullptr);
	reader->requestScalarProperty("radius", "radius", PlyReader::TYPE_DOUBLE, 0);

	reader->requestElement(
		"material",
		std::bind(
			&Fem1DRepresentationPlyReaderDelegate::beginMaterials, this, std::placeholders::_1, std::placeholders::_2),
		nullptr,
		nullptr);
	reader->requestScalarProperty(
		"material", "mass_density", PlyReader::TYPE_DOUBLE, offsetof(MaterialData, massDensity));
	reader->requestScalarProperty(
		"material", "poisson_ratio", PlyReader::TYPE_DOUBLE, offsetof(MaterialData, poissonRatio));
	reader->requestScalarProperty(
		"material", "young_modulus", PlyReader::TYPE_DOUBLE, offsetof(MaterialData, youngModulus));

	reader->setStartParseFileCallback(std::bind(&Fem1DRepresentationPlyReaderDelegate::startParseFile, this));
	reader->setEndParseFileCallback(std::bind(&Fem1DRepresentationPlyReaderDelegate::endParseFile, this));

	return true;
}

bool Fem1DRepresentationPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = true;

	// Shortcut test if one fails ...
	result = result && reader.hasProperty("vertex", "x");
	result = result && reader.hasProperty("vertex", "y");
	result = result && reader.hasProperty("vertex", "z");

	result = result && reader.hasProperty("1DElement", "vertex_indices");
	result = result && !reader.isScalar("1DElement", "vertex_indices");

	result = result && reader.hasProperty("radius", "radius");

	result = result && reader.hasProperty("material", "mass_density");
	result = result && reader.hasProperty("material", "poisson_ratio");
	result = result && reader.hasProperty("material", "young_modulus");

	m_hasBoundaryConditions = reader.hasProperty("boundary_condition", "vertex_index");

	return result;
}

void Fem1DRepresentationPlyReaderDelegate::processFemElement(const std::string& elementName)
{
	SURGSIM_ASSERT(m_femData.vertexCount == 2) << "Cannot process 1DElement with "
											   << m_femData.vertexCount << " vertices.";

	std::array<size_t, 2> triangleVertices;
	std::copy(m_femData.indicies, m_femData.indicies + 2, triangleVertices.begin());
	m_fem->addFemElement(std::make_shared<Fem1DElementBeam>(triangleVertices));
}

void* Fem1DRepresentationPlyReaderDelegate::beginRadius(const std::string& elementName, size_t thicknessCount)
{
	return &m_radius;
}

void Fem1DRepresentationPlyReaderDelegate::endParseFile()
{
	for (size_t i = 0; i < m_fem->getNumFemElements(); ++i)
	{
		std::static_pointer_cast<Fem1DElementBeam>(m_fem->getFemElement(i))->setRadius(m_radius);
	}

	FemRepresentationPlyReaderDelegate::endParseFile();
}

}; // namespace Physics
}; // namespace SurgSim