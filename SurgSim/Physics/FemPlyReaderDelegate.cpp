// This file is a part of the OpenSurgSim project.
// Copyright 2014-2015, SimQuest Solutions Inc.
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
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/FemPlyReaderDelegate.h"

using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Physics
{

FemPlyReaderDelegate::FemPlyReaderDelegate()
{
}

bool FemPlyReaderDelegate::registerDelegate(PlyReader* reader)
{
	// Element Processing
	reader->requestElement(
		getElementName(),
		std::bind(&FemPlyReaderDelegate::beginFemElements,
				  this,
				  std::placeholders::_1,
				  std::placeholders::_2),
		std::bind(&FemPlyReaderDelegate::processFemElement, this, std::placeholders::_1),
		std::bind(&FemPlyReaderDelegate::endFemElements, this, std::placeholders::_1));

	reader->requestListProperty(getElementName(),
								"vertex_indices",
								PlyReader::TYPE_UNSIGNED_INT,
								offsetof(ElementData, indices),
								PlyReader::TYPE_UNSIGNED_INT,
								offsetof(ElementData, vertexCount));

	// Vertex processing
	reader->requestElement("vertex",
						   std::bind(&FemPlyReaderDelegate::beginVertices, this,
									 std::placeholders::_1, std::placeholders::_2),
						   std::bind(&FemPlyReaderDelegate::processVertex, this, std::placeholders::_1),
						   std::bind(&FemPlyReaderDelegate::endVertices, this, std::placeholders::_1));

	reader->requestScalarProperty("vertex", "x", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, x));
	reader->requestScalarProperty("vertex", "y", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, y));
	reader->requestScalarProperty("vertex", "z", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, z));

	if (m_hasRotationDOF)
	{
		reader->requestScalarProperty("vertex", "thetaX", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, thetaX));
		reader->requestScalarProperty("vertex", "thetaY", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, thetaY));
		reader->requestScalarProperty("vertex", "thetaZ", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, thetaZ));
	}


	// Boundary Condition Processing
	if (m_hasBoundaryConditions)
	{
		reader->requestElement(
			"boundary_condition",
			std::bind(&FemPlyReaderDelegate::beginBoundaryConditions,
					  this,
					  std::placeholders::_1,
					  std::placeholders::_2),
			std::bind(&FemPlyReaderDelegate::processBoundaryCondition, this, std::placeholders::_1),
			nullptr);
		reader->requestScalarProperty("boundary_condition", "vertex_index", PlyReader::TYPE_UNSIGNED_INT, 0);
	}


	if (m_hasMaterial)
	{
		if (m_hasPerElementMaterial)
		{
			reader->requestScalarProperty(
					getElementName(), "mass_density", PlyReader::TYPE_DOUBLE, offsetof(ElementData, massDensity));
			reader->requestScalarProperty(
					getElementName(), "poisson_ratio", PlyReader::TYPE_DOUBLE, offsetof(ElementData, poissonRatio));
			reader->requestScalarProperty(
					getElementName(), "young_modulus", PlyReader::TYPE_DOUBLE, offsetof(ElementData, youngModulus));
		}
		else
		{
			reader->requestElement(
					"material",
					std::bind(
							&FemPlyReaderDelegate::beginMaterials, this, std::placeholders::_1, std::placeholders::_2),
					nullptr,
					std::bind(&FemPlyReaderDelegate::endMaterials, this, std::placeholders::_1));
			reader->requestScalarProperty(
					"material", "mass_density", PlyReader::TYPE_DOUBLE, offsetof(MaterialData, massDensity));
			reader->requestScalarProperty(
					"material", "poisson_ratio", PlyReader::TYPE_DOUBLE, offsetof(MaterialData, poissonRatio));
			reader->requestScalarProperty(
					"material", "young_modulus", PlyReader::TYPE_DOUBLE, offsetof(MaterialData, youngModulus));
		}
	}

	reader->setEndParseFileCallback(std::bind(&FemPlyReaderDelegate::endParseFile, this));

	return true;
}

bool FemPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = true;

	// Shortcut test if one fails ...
	result = result && reader.hasElement(getElementName());

	result = result && reader.hasProperty("vertex", "x");
	result = result && reader.hasProperty("vertex", "y");
	result = result && reader.hasProperty("vertex", "z");

	// 6DOF processing (this is not supported for all element types)
	m_hasRotationDOF = reader.hasProperty("vertex", "thetaX") &&
					   reader.hasProperty("vertex", "thetaY") &&
					   reader.hasProperty("vertex", "thetaZ");

	result = result && reader.hasProperty(getElementName(), "vertex_indices");
	result = result && !reader.isScalar(getElementName(), "vertex_indices");

	// Material: either have a default material for all elements
	// or have per element material properties
	m_hasMaterial = reader.hasProperty("material", "mass_density") &&
					   reader.hasProperty("material", "poisson_ratio") &&
					   reader.hasProperty("material", "young_modulus");

	m_hasPerElementMaterial = reader.hasProperty(getElementName(), "mass_density") &&
							  reader.hasProperty(getElementName(), "poisson_ratio") &&
							  reader.hasProperty(getElementName(), "young_modulus");

	m_hasMaterial = m_hasMaterial || m_hasPerElementMaterial;

	m_hasBoundaryConditions = reader.hasProperty("boundary_condition", "vertex_index");

	return result;
}

void* FemPlyReaderDelegate::beginVertices(const std::string& elementName, size_t vertexCount)
{
	m_vertexData.overrun1 = 0l;
	return &m_vertexData;
}

void FemPlyReaderDelegate::endVertices(const std::string& elementName)
{
	SURGSIM_ASSERT(m_vertexData.overrun1 == 0l) <<
			"There was an overrun while reading the vertex structures, it is likely that data " <<
			"has become corrupted.";
}

void* FemPlyReaderDelegate::beginFemElements(const std::string& elementName, size_t elementCount)
{
	m_elementData.overrun1 = 0l;
	m_elementData.overrun2 = 0l;
	return &m_elementData;
}

void FemPlyReaderDelegate::endFemElements(const std::string& elementName)
{
	SURGSIM_ASSERT(m_elementData.overrun1 == 0 && m_elementData.overrun2 == 0) <<
			"There was an overrun while reading the element structures, it is likely that data " <<
			"has become corrupted.";
	m_elementData.indices = nullptr;
}

void* FemPlyReaderDelegate::beginMaterials(const std::string& elementName, size_t materialCount)
{
	m_materialData.overrun = 0l;
	return &m_materialData;
}

void FemPlyReaderDelegate::endMaterials(const std::string& elementName)
{
	SURGSIM_ASSERT(m_materialData.overrun == 0) <<
			"There was an overrun while reading the material structures, it is likely that data " <<
			"has become corrupted.";
}

void* FemPlyReaderDelegate::beginBoundaryConditions(const std::string& elementName,
		size_t boundaryConditionCount)
{
	return &m_boundaryConditionData;
}

} // namespace SurgSim
} // namespace DataStructures
