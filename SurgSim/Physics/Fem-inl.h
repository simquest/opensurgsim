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

#ifndef SURGSIM_PHYSICS_FEM_INL_H
#define SURGSIM_PHYSICS_FEM_INL_H

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/Log.h"

using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Physics
{

template <class VertexData, class Element>
Fem<VertexData, Element>::Fem()
{
}

template <class VertexData, class Element>
size_t Fem<VertexData, Element>::addElement(std::shared_ptr<Element> element)
{
	m_elements.push_back(element);
	return m_elements.size() - 1;
}

template <class VertexData, class Element>
size_t Fem<VertexData, Element>::getNumElements() const
{
	return m_elements.size();
}

template <class VertexData, class Element>
const std::vector<std::shared_ptr<Element>>&
	Fem<VertexData, Element>::getElements() const
{
	return m_elements;
}

template <class VertexData, class Element>
std::vector<std::shared_ptr<Element>>& Fem<VertexData, Element>::getElements()
{
	return m_elements;
}

template <class VertexData, class Element>
std::shared_ptr<Element> Fem<VertexData, Element>::getElement(size_t id) const
{
	return m_elements[id];
}

template <class VertexData, class Element>
size_t Fem<VertexData, Element>::addBoundaryCondition(size_t boundaryCondition)
{
	m_boundaryConditions.push_back(boundaryCondition);
	return m_boundaryConditions.size() - 1;
}

template <class VertexData, class Element>
const std::vector<size_t>& Fem<VertexData, Element>::getBoundaryConditions() const
{
	return m_boundaryConditions;
}

template <class VertexData, class Element>
std::vector<size_t>& Fem<VertexData, Element>::getBoundaryConditions()
{
	return m_boundaryConditions;
}

template <class VertexData, class Element>
size_t Fem<VertexData, Element>::getBoundaryCondition(size_t id) const
{
	return m_boundaryConditions[id];
}
template <class VertexData, class Element> template <class PlyType, class FemType>
bool Fem<VertexData, Element>::loadFemFile(const std::string& filename)
{
	SurgSim::DataStructures::PlyReader reader(filename);
	if (!reader.isValid())
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
			<< "'" << filename << "' is an invalid .ply file.";
		return false;
	}

	auto delegate = std::make_shared<PlyType>(
				std::dynamic_pointer_cast<FemType>(this->shared_from_this()));
	if (!reader.parseWithDelegate(delegate))
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
			<< "The input file '" << filename << "' does not have the property required by FEM element mesh.";
		return false;
	}

	return true;
}

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM_INL_H
