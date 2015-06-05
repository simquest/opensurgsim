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
#include "SurgSim/Physics/Fem2DElementTriangle.h"
#include "SurgSim/Physics/Fem2DPlyReaderDelegate.h"


using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Physics
{

Fem2DPlyReaderDelegate::Fem2DPlyReaderDelegate()
{

}

Fem2DPlyReaderDelegate::Fem2DPlyReaderDelegate(std::shared_ptr<Fem2D> mesh) :
	m_mesh(mesh)
{
	SURGSIM_ASSERT(mesh != nullptr) << "The mesh cannot be null.";
	mesh->clear();
}

std::string Fem2DPlyReaderDelegate::getElementName() const
{
	return "2d_element";
}

bool Fem2DPlyReaderDelegate::registerDelegate(PlyReader* reader)
{
	// Vertex processing
	reader->requestElement("vertex",
						   std::bind(&Fem2DPlyReaderDelegate::beginVertices, this,
									 std::placeholders::_1, std::placeholders::_2),
						   std::bind(&Fem2DPlyReaderDelegate::processVertex, this, std::placeholders::_1),
						   std::bind(&Fem2DPlyReaderDelegate::endVertices, this, std::placeholders::_1));
	reader->requestScalarProperty("vertex", "x", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, x));
	reader->requestScalarProperty("vertex", "y", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, y));
	reader->requestScalarProperty("vertex", "z", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, z));

	if (m_hasRotationDOF)
	{
		reader->requestScalarProperty("vertex", "thetaX", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, thetaX));
		reader->requestScalarProperty("vertex", "thetaY", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, thetaY));
		reader->requestScalarProperty("vertex", "thetaZ", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, thetaZ));
	}

	// Thickness processing

	reader->requestElement(
		"thickness",
		std::bind(
		&Fem2DPlyReaderDelegate::beginThickness, this, std::placeholders::_1, std::placeholders::_2),
		nullptr,
		std::bind(&Fem2DPlyReaderDelegate::endThickness, this, std::placeholders::_1));
	reader->requestScalarProperty("thickness", "value", PlyReader::TYPE_DOUBLE, 0);

	FemPlyReaderDelegate::registerDelegate(reader);

	return true;
}

bool Fem2DPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = FemPlyReaderDelegate::fileIsAcceptable(reader);

	// 6DOF processing
	m_hasRotationDOF = reader.hasProperty("vertex", "thetaX") && reader.hasProperty("vertex", "thetaY") &&
							  reader.hasProperty("vertex", "thetaZ");

	result = result && reader.hasProperty("thickness", "value");

	return result;
}

void Fem2DPlyReaderDelegate::endParseFile()
{
	for(auto element : m_mesh->getElements())
	{
		element->data->thickness = m_thickness;
		element->data->massDensity = m_materialData.massDensity;
		element->data->poissonRatio = m_materialData.poissonRatio;
		element->data->youngModulus = m_materialData.youngModulus;
	}
	m_mesh->update();
}

void* Fem2DPlyReaderDelegate::beginVertices(const std::string &elementName, size_t vertexCount)
{
	m_vertexData.overrun1 = 0l;
	m_vertexData.overrun2 = 0l;
	return &m_vertexData;
}

void Fem2DPlyReaderDelegate::processVertex(const std::string& elementName)
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

	Fem2D::VertexType vertex(SurgSim::Math::Vector3d(m_vertexData.x, m_vertexData.y, m_vertexData.z), data);

	m_mesh->addVertex(vertex);
}

void Fem2DPlyReaderDelegate::endVertices(const std::string &elementName)
{
	SURGSIM_ASSERT(m_vertexData.overrun1 == 0l && m_vertexData.overrun2 == 0l) <<
			"There was an overrun while reading the vertex structures, it is likely that data " <<
			"has become corrupted.";
}

void Fem2DPlyReaderDelegate::processFemElement(const std::string& elementName)
{
	SURGSIM_ASSERT(m_elementData.vertexCount == 3) << "Cannot process 2D Element with "
		<< m_elementData.vertexCount << " vertices.";

	std::array<size_t, 3> nodes;
	std::copy(m_elementData.indices, m_elementData.indices + m_elementData.vertexCount, nodes.data());
	auto data = std::make_shared<FemElementStructs::FemElement2DParameter>();
	static Fem2DElementTriangle triangle;
	data->type = triangle.getClassName();
	auto femElement = std::make_shared<TriangleType>(nodes, data);
	m_mesh->addElement(femElement);
}

void* Fem2DPlyReaderDelegate::beginThickness(const std::string& elementName, size_t thicknessCount)
{
	return &m_thickness;
}

void Fem2DPlyReaderDelegate::endThickness(const std::string &elementName)
{
	SURGSIM_ASSERT(SurgSim::Math::isValid(m_thickness)) << "No radius information processed.";
}

void Fem2DPlyReaderDelegate::processBoundaryCondition(const std::string& elementName)
{
	m_mesh->addBoundaryCondition(static_cast<size_t>(m_boundaryConditionData));
}

} // namespace Physics
} // namespace SurgSim
