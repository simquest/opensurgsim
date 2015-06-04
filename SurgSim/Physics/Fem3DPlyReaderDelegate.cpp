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
#include "SurgSim/Physics/Fem3DPlyReaderDelegate.h"


using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Physics
{

Fem3DPlyReaderDelegate::Fem3DPlyReaderDelegate()
{

}

Fem3DPlyReaderDelegate::Fem3DPlyReaderDelegate(std::shared_ptr<Fem3D> mesh) :
	m_mesh(mesh)
{
	SURGSIM_ASSERT(mesh != nullptr) << "The mesh cannot be null.";
	mesh->clear();
}

bool Fem3DPlyReaderDelegate::registerDelegate(PlyReader* reader)
{
	// Vertex processing
	reader->requestElement("vertex",
						   std::bind(&Fem3DPlyReaderDelegate::beginVertices, this,
									 std::placeholders::_1, std::placeholders::_2),
						   std::bind(&Fem3DPlyReaderDelegate::processVertex, this, std::placeholders::_1),
						   std::bind(&Fem3DPlyReaderDelegate::endVertices, this, std::placeholders::_1));
	reader->requestScalarProperty("vertex", "x", PlyReader::TYPE_DOUBLE, offsetof(VertexData, x));
	reader->requestScalarProperty("vertex", "y", PlyReader::TYPE_DOUBLE, offsetof(VertexData, y));
	reader->requestScalarProperty("vertex", "z", PlyReader::TYPE_DOUBLE, offsetof(VertexData, z));

	// Element Processing
	reader->requestElement(
		"3d_element",
		std::bind(&Fem3DPlyReaderDelegate::beginFemElements,
		this,
		std::placeholders::_1,
		std::placeholders::_2),
		std::bind(&Fem3DPlyReaderDelegate::processFemElement, this, std::placeholders::_1),
		std::bind(&Fem3DPlyReaderDelegate::endFemElements, this, std::placeholders::_1));
	reader->requestListProperty("3d_element",
		"vertex_indices",
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(FemElement3D, indices),
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(FemElement3D, vertexCount));

	// Boundary Condition processing

	m_hasBoundaryConditions = reader->hasProperty("boundary_condition", "vertex_index");

	if (m_hasBoundaryConditions)
	{
		reader->requestElement(
			"boundary_condition",
			std::bind(&Fem3DPlyReaderDelegate::beginBoundaryConditions,
			this,
			std::placeholders::_1,
			std::placeholders::_2),
			std::bind(&Fem3DPlyReaderDelegate::processBoundaryCondition, this, std::placeholders::_1),
			nullptr);
		reader->requestScalarProperty("boundary_condition", "vertex_index", PlyReader::TYPE_UNSIGNED_INT, 0);
	}

	// Material processing

	reader->requestElement(
		"material",
		std::bind(
			&Fem3DPlyReaderDelegate::beginMaterials, this, std::placeholders::_1, std::placeholders::_2),
		nullptr,
		std::bind(&Fem3DPlyReaderDelegate::endMaterials, this, std::placeholders::_1));
	reader->requestScalarProperty("material", "mass_density", PlyReader::TYPE_DOUBLE, offsetof(Material, massDensity));
	reader->requestScalarProperty("material", "poisson_ratio",
								  PlyReader::TYPE_DOUBLE, offsetof(Material, poissonRatio));
	reader->requestScalarProperty("material", "young_modulus",
								  PlyReader::TYPE_DOUBLE, offsetof(Material, youngModulus));

	reader->setEndParseFileCallback(std::bind(&Fem3DPlyReaderDelegate::endFile, this));

	return true;
}

bool Fem3DPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = true;

	// Shortcut test if one fails ...
	result = result && reader.hasProperty("vertex", "x");
	result = result && reader.hasProperty("vertex", "y");
	result = result && reader.hasProperty("vertex", "z");

	//result = result && reader.hasProperty("3d_element", "type");
	result = result && reader.hasProperty("3d_element", "vertex_indices");
	result = result && !reader.isScalar("3d_element", "vertex_indices");

	result = result && reader.hasProperty("material", "mass_density");
	result = result && reader.hasProperty("material", "poisson_ratio");
	result = result && reader.hasProperty("material", "young_modulus");

	return result;
}

void* Fem3DPlyReaderDelegate::beginVertices(const std::string &elementName, size_t vertexCount)
{
	m_vertexData.overrun1 = 0l;
	return &m_vertexData;
}

void Fem3DPlyReaderDelegate::processVertex(const std::string& elementName)
{
	Fem3D::VertexType vertex(SurgSim::Math::Vector3d(m_vertexData.x, m_vertexData.y, m_vertexData.z));

	m_mesh->addVertex(vertex);
}

void Fem3DPlyReaderDelegate::endVertices(const std::string &elementName)
{
	SURGSIM_ASSERT(m_vertexData.overrun1 == 0l) <<
			"There was an overrun while reading the vertex structures, it is likely that data " <<
			"has become corrupted.";
}

void* Fem3DPlyReaderDelegate::beginFemElements(const std::string& elementName, size_t elementCount)
{
	m_elementData.overrun1 = 0l;
	m_elementData.overrun2 = 0l;
	return &m_elementData;
}

void Fem3DPlyReaderDelegate::processFemElement(const std::string& elementName)
{
	SURGSIM_ASSERT(m_elementData.vertexCount == 4 || m_elementData.vertexCount == 8) <<
			"Cannot process 3D Element with " << m_elementData.vertexCount << " vertices.";

	auto data = std::make_shared<FemElementStructs::FemElement3DParameter>();
	if (m_elementData.vertexCount == 8)
	{
		std::array<size_t, 8> nodes;
		std::copy(m_elementData.indices, m_elementData.indices + m_elementData.vertexCount, nodes.data());
		data->type = "SurgSim::Physics::Fem3DElementCube";
		auto femElement = std::make_shared<CubeType>(nodes, data);
		m_mesh->addCube(femElement);
	}
	else {
		std::array<size_t, 4> nodes;
		std::copy(m_elementData.indices, m_elementData.indices + m_elementData.vertexCount, nodes.data());
		data->type = "SurgSim::Physics::Fem3DElementTetrahedron";
		auto femElement = std::make_shared<TetrahedronType>(nodes, data);
		m_mesh->addElement(femElement);
	}
}

void Fem3DPlyReaderDelegate::endFemElements(const std::string& elementName)
{
	SURGSIM_ASSERT(m_elementData.overrun1 == 0 && m_elementData.overrun2 == 0) <<
		"There was an overrun while reading the element structures, it is likely that data " <<
		"has become corrupted.";
	m_elementData.indices = nullptr;
}

void* Fem3DPlyReaderDelegate::beginMaterials(const std::string& elementName, size_t materialCount)
{
	m_materialData.overrun = 0l;
	return &m_materialData;
}

void Fem3DPlyReaderDelegate::endMaterials(const std::string& elementName)
{
	SURGSIM_ASSERT(m_materialData.overrun == 0) <<
		"There was an overrun while reading the material structures, it is likely that data " <<
		"has become corrupted.";
}

void* Fem3DPlyReaderDelegate::beginBoundaryConditions(const std::string& elementName,
																  size_t boundaryConditionCount)
{
	return &m_boundaryConditionData;
}

void Fem3DPlyReaderDelegate::processBoundaryCondition(const std::string& elementName)
{
	m_mesh->addBoundaryCondition(static_cast<size_t>(m_boundaryConditionData));
}

void Fem3DPlyReaderDelegate::endFile()
{
	for(auto element : m_mesh->getElements())
	{
		element->data->massDensity = m_materialData.massDensity;
		element->data->poissonRatio = m_materialData.poissonRatio;
		element->data->youngModulus = m_materialData.youngModulus;
	}

	for(auto element : m_mesh->getCubes())
	{
		element->data->massDensity = m_materialData.massDensity;
		element->data->poissonRatio = m_materialData.poissonRatio;
		element->data->youngModulus = m_materialData.youngModulus;
	}
	m_mesh->update();
}

} // namespace Physics
} // namespace SurgSim
