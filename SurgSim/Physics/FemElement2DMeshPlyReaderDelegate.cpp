// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#include "SurgSim/Math/Valid.h"
#include "SurgSim/Physics/FemElement2DMeshPlyReaderDelegate.h"


using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Physics
{

FemElement2DMeshPlyReaderDelegate::FemElement2DMeshPlyReaderDelegate()
{

}

FemElement2DMeshPlyReaderDelegate::FemElement2DMeshPlyReaderDelegate(std::shared_ptr<FemElement2DMesh> mesh) :
	m_mesh(mesh)
{
	SURGSIM_ASSERT(mesh != nullptr) << "The mesh cannot be null.";
	mesh->clear();
}

bool FemElement2DMeshPlyReaderDelegate::registerDelegate(PlyReader* reader)
{
	// Vertex processing
	reader->requestElement("vertex",
						   std::bind(&FemElement2DMeshPlyReaderDelegate::beginVertices, this,
									 std::placeholders::_1, std::placeholders::_2),
						   std::bind(&FemElement2DMeshPlyReaderDelegate::processVertex, this, std::placeholders::_1),
						   std::bind(&FemElement2DMeshPlyReaderDelegate::endVertices, this, std::placeholders::_1));
	reader->requestScalarProperty("vertex", "x", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, x));
	reader->requestScalarProperty("vertex", "y", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, y));
	reader->requestScalarProperty("vertex", "z", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, z));

	// 6DOF processing
	m_hasRotationDOF = reader->hasProperty("vertex", "thetaX") && reader->hasProperty("vertex", "thetaY") &&
							  reader->hasProperty("vertex", "thetaZ");

	if (m_hasRotationDOF)
	{
		reader->requestScalarProperty("vertex", "thetaX", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, thetaX));
		reader->requestScalarProperty("vertex", "thetaY", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, thetaY));
		reader->requestScalarProperty("vertex", "thetaZ", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, thetaZ));
	}

	// Element Processing
	reader->requestElement(
		"2d_element",
		std::bind(&FemElement2DMeshPlyReaderDelegate::beginFemElements,
		this,
		std::placeholders::_1,
		std::placeholders::_2),
		std::bind(&FemElement2DMeshPlyReaderDelegate::processFemElement, this, std::placeholders::_1),
		std::bind(&FemElement2DMeshPlyReaderDelegate::endFemElements, this, std::placeholders::_1));
	reader->requestListProperty("2d_element",
		"vertex_indices",
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(FemElement2D, indices),
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(FemElement2D, vertexCount));

	// Boundary Condition processing

	m_hasBoundaryConditions = reader->hasProperty("boundary_condition", "vertex_index");

	if (m_hasBoundaryConditions)
	{
		reader->requestElement(
			"boundary_condition",
			std::bind(&FemElement2DMeshPlyReaderDelegate::beginBoundaryConditions,
			this,
			std::placeholders::_1,
			std::placeholders::_2),
			std::bind(&FemElement2DMeshPlyReaderDelegate::processBoundaryCondition, this, std::placeholders::_1),
			nullptr);
		reader->requestScalarProperty("boundary_condition", "vertex_index", PlyReader::TYPE_UNSIGNED_INT, 0);
	}

	// Thickness processing

	reader->requestElement(
		"thickness",
		std::bind(
		&FemElement2DMeshPlyReaderDelegate::beginThickness, this, std::placeholders::_1, std::placeholders::_2),
		nullptr,
		std::bind(&FemElement2DMeshPlyReaderDelegate::endThickness, this, std::placeholders::_1));
	reader->requestScalarProperty("thickness", "value", PlyReader::TYPE_DOUBLE, 0);

	// Material processing

	reader->requestElement(
		"material",
		std::bind(
			&FemElement2DMeshPlyReaderDelegate::beginMaterials, this, std::placeholders::_1, std::placeholders::_2),
		nullptr,
		std::bind(&FemElement2DMeshPlyReaderDelegate::endMaterials, this, std::placeholders::_1));
	reader->requestScalarProperty("material", "mass_density", PlyReader::TYPE_DOUBLE, offsetof(Material, massDensity));
	reader->requestScalarProperty("material", "poisson_ratio",
								  PlyReader::TYPE_DOUBLE, offsetof(Material, poissonRatio));
	reader->requestScalarProperty("material", "young_modulus",
								  PlyReader::TYPE_DOUBLE, offsetof(Material, youngModulus));

	reader->setEndParseFileCallback(std::bind(&FemElement2DMeshPlyReaderDelegate::endFile, this));

	return true;
}

bool FemElement2DMeshPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = true;

	// Shortcut test if one fails ...
	result = result && reader.hasProperty("vertex", "x");
	result = result && reader.hasProperty("vertex", "y");
	result = result && reader.hasProperty("vertex", "z");

	//result = result && reader.hasProperty("2d_element", "type");
	result = result && reader.hasProperty("2d_element", "vertex_indices");
	result = result && !reader.isScalar("2d_element", "vertex_indices");

	result = result && reader.hasProperty("thickness", "value");

	result = result && reader.hasProperty("material", "mass_density");
	result = result && reader.hasProperty("material", "poisson_ratio");
	result = result && reader.hasProperty("material", "young_modulus");

	return result;
}

void* FemElement2DMeshPlyReaderDelegate::beginVertices(const std::string &elementName, size_t vertexCount)
{
	m_vertexData.overrun1 = 0l;
	m_vertexData.overrun2 = 0l;
	return &m_vertexData;
}

void FemElement2DMeshPlyReaderDelegate::processVertex(const std::string& elementName)
{
	FemElementStructs::RotationVectorData data;

	if (m_hasRotationDOF)
	{
		data.thetaX = m_vertexData.thetaX;
		data.thetaY = m_vertexData.thetaY;
		data.thetaZ = m_vertexData.thetaZ;
	}
	else
	{
		data.thetaX = 0.0;
		data.thetaY = 0.0;
		data.thetaZ = 0.0;
	}

	FemElement2DMesh::VertexType vertex(SurgSim::Math::Vector3d(m_vertexData.x, m_vertexData.y, m_vertexData.z), data);

	m_mesh->addVertex(vertex);
}

void FemElement2DMeshPlyReaderDelegate::endVertices(const std::string &elementName)
{
	SURGSIM_ASSERT(m_vertexData.overrun1 == 0l && m_vertexData.overrun2 == 0l) <<
			"There was an overrun while reading the vertex structures, it is likely that data " <<
			"has become corrupted.";
}

void* FemElement2DMeshPlyReaderDelegate::beginFemElements(const std::string& elementName, size_t elementCount)
{
	m_elementData.overrun1 = 0l;
	m_elementData.overrun2 = 0l;
	return &m_elementData;
}

void FemElement2DMeshPlyReaderDelegate::processFemElement(const std::string& elementName)
{
	SURGSIM_ASSERT(m_elementData.vertexCount == 3) << "Cannot process 2D Element with "
		<< m_elementData.vertexCount << " vertices.";

	auto femElement = std::make_shared<FemElementStructs::FemElement2D>();
	femElement->type = "SurgSim::Physics::Fem2DElementTriangle";
	femElement->nodeIds.resize(m_elementData.vertexCount);
	std::copy(m_elementData.indices, m_elementData.indices + m_elementData.vertexCount, femElement->nodeIds.data());
	m_mesh->addFemElement(femElement);
}

void FemElement2DMeshPlyReaderDelegate::endFemElements(const std::string& elementName)
{
	SURGSIM_ASSERT(m_elementData.overrun1 == 0 && m_elementData.overrun2 == 0) <<
		"There was an overrun while reading the element structures, it is likely that data " <<
		"has become corrupted.";
	m_elementData.indices = nullptr;
}

void* FemElement2DMeshPlyReaderDelegate::beginThickness(const std::string& elementName, size_t thicknessCount)
{
	return &m_thickness;
}

void FemElement2DMeshPlyReaderDelegate::endThickness(const std::string &elementName)
{
	SURGSIM_ASSERT(SurgSim::Math::isValid(m_thickness)) << "No radius information processed.";
}

void* FemElement2DMeshPlyReaderDelegate::beginMaterials(const std::string& elementName, size_t materialCount)
{
	m_materialData.overrun = 0l;
	return &m_materialData;
}

void FemElement2DMeshPlyReaderDelegate::endMaterials(const std::string& elementName)
{
	SURGSIM_ASSERT(m_materialData.overrun == 0) <<
		"There was an overrun while reading the material structures, it is likely that data " <<
		"has become corrupted.";
}

void* FemElement2DMeshPlyReaderDelegate::beginBoundaryConditions(const std::string& elementName,
																  size_t boundaryConditionCount)
{
	return &m_boundaryConditionData;
}

void FemElement2DMeshPlyReaderDelegate::processBoundaryCondition(const std::string& elementName)
{
	m_mesh->addBoundaryCondition(static_cast<size_t>(m_boundaryConditionData));
}

void FemElement2DMeshPlyReaderDelegate::endFile()
{
	for(auto element : m_mesh->getFemElements())
	{
		element->thickness = m_thickness;
		element->massDensity = m_materialData.massDensity;
		element->poissonRatio = m_materialData.poissonRatio;
		element->youngModulus = m_materialData.youngModulus;
	}
	m_mesh->update();
}

} // namespace Physics
} // namespace SurgSim
