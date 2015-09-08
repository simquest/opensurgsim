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

std::string Fem3DPlyReaderDelegate::getElementName() const
{
	return "3d_element";
}

bool Fem3DPlyReaderDelegate::registerDelegate(PlyReader* reader)
{
	// Vertex processing
	reader->requestElement("vertex",
						   std::bind(&Fem3DPlyReaderDelegate::beginVertices, this,
									 std::placeholders::_1, std::placeholders::_2),
						   std::bind(&Fem3DPlyReaderDelegate::processVertex, this, std::placeholders::_1),
						   std::bind(&Fem3DPlyReaderDelegate::endVertices, this, std::placeholders::_1));
	reader->requestScalarProperty("vertex", "x", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, x));
	reader->requestScalarProperty("vertex", "y", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, y));
	reader->requestScalarProperty("vertex", "z", PlyReader::TYPE_DOUBLE, offsetof(Vertex6DData, z));

	FemPlyReaderDelegate::registerDelegate(reader);

	return true;
}

bool Fem3DPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = FemPlyReaderDelegate::fileIsAcceptable(reader);

	SURGSIM_ASSERT(!reader.hasProperty("vertex", "thetaX") &&
		!reader.hasProperty("vertex", "thetaY") && !reader.hasProperty("vertex", "thetaZ")) <<
		"An Fem3d model cannot contain rotational data (only 3 translational degrees of freedom)";

	return result;
}

void Fem3DPlyReaderDelegate::endParseFile()
{
	for(auto element : m_mesh->getElements())
	{
		element->massDensity = m_materialData.massDensity;
		element->poissonRatio = m_materialData.poissonRatio;
		element->youngModulus = m_materialData.youngModulus;
	}

	m_mesh->update();
}

void Fem3DPlyReaderDelegate::processVertex(const std::string& elementName)
{
	Fem3D::VertexType vertex(SurgSim::Math::Vector3d(m_vertexData.x, m_vertexData.y, m_vertexData.z));

	m_mesh->addVertex(vertex);
}

void Fem3DPlyReaderDelegate::processFemElement(const std::string& elementName)
{
	SURGSIM_ASSERT(m_elementData.vertexCount == 4 || m_elementData.vertexCount == 8) <<
			"Cannot process 3D Element with " << m_elementData.vertexCount << " vertices.";

	auto femElement = std::make_shared<FemElementStructs::FemElement3DParameter>();
	femElement->nodeIds.resize(m_elementData.vertexCount);
	std::copy(m_elementData.indices, m_elementData.indices + m_elementData.vertexCount, femElement->nodeIds.data());
	m_mesh->addElement(femElement);
}

void Fem3DPlyReaderDelegate::processBoundaryCondition(const std::string& elementName)
{
	m_mesh->addBoundaryCondition(static_cast<size_t>(m_boundaryConditionData));
}

} // namespace Physics
} // namespace SurgSim
