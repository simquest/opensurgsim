// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
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

#include <SurgSim/Physics/FemElement.h>

namespace SurgSim
{

namespace Physics
{

FemElement::FemElement() : m_numDofPerNode(0), m_rho(0.0)
{}

FemElement::~FemElement()
{}

unsigned int FemElement::getNumDofPerNode() const
{
	return m_numDofPerNode;
}

void FemElement::setNumDofPerNode(unsigned int numDofPerNode)
{
	m_numDofPerNode = numDofPerNode;
}

unsigned int FemElement::getNumNodes() const
{
	return m_nodeIds.size();
}

unsigned int FemElement::getNodeId(unsigned int elementNodeId) const
{
	return m_nodeIds[elementNodeId];
}

const std::vector<unsigned int>& FemElement::getNodeIds() const
{
	return m_nodeIds;
}

void FemElement::setMassDensity(double rho)
{
	m_rho = rho;
}

double FemElement::getMassDensity() const
{
	return m_rho;
}

double FemElement::getMass(const DeformableRepresentationState& state) const
{
	return getVolume(state) * m_rho;
}

} // namespace Physics

} // namespace SurgSim
