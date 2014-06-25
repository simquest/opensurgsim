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

#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/FemRepresentation.h"
#include "SurgSim/Physics/FemRepresentationPlyReaderDelegate.h"

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

FemRepresentationPlyReaderDelegate::ElementData::ElementData() : indicies(nullptr), vertexCount(0)
{
}

void FemRepresentationPlyReaderDelegate::startParseFile()
{
	SURGSIM_ASSERT(nullptr != m_fem) << "The Representation cannot be nullptr.";
	SURGSIM_ASSERT(0 == m_fem->getNumFemElements())
		<< "The Representation already contains fem elements, so it cannot be initialized.";
	SURGSIM_ASSERT(nullptr == m_fem->getInitialState()) << "The Representation's initial state must be uninitialized.";

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
	m_femData.indicies = nullptr;
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
