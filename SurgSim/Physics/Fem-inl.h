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

template <class VertexData, class EdgeData, class TriangleData, class Element>
Fem<VertexData, EdgeData, TriangleData, Element>::Fem()
{
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
size_t Fem<VertexData, EdgeData, TriangleData, Element>::addFemElement(std::shared_ptr<Element> element)
{
	m_femElements.push_back(element);
	return m_femElements.size() - 1;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
size_t Fem<VertexData, EdgeData, TriangleData, Element>::getNumElements() const
{
	return m_femElements.size();
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
const std::vector<std::shared_ptr<Element>>&
	Fem<VertexData, EdgeData, TriangleData, Element>::getFemElements() const
{
	return m_femElements;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
std::vector<std::shared_ptr<Element>>& Fem<VertexData, EdgeData, TriangleData, Element>::getFemElements()
{
	return m_femElements;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
std::shared_ptr<Element> Fem<VertexData, EdgeData, TriangleData, Element>::getFemElement(size_t id) const
{
	return m_femElements[id];
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
void Fem<VertexData, EdgeData, TriangleData, Element>::removeFemElement(size_t id)
{
	m_femElements.erase(m_femElements.begin() + id);
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
size_t Fem<VertexData, EdgeData, TriangleData, Element>::addBoundaryCondition(size_t boundaryCondition)
{
	m_boundaryConditions.push_back(boundaryCondition);
	return m_boundaryConditions.size() - 1;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
const std::vector<size_t>& Fem<VertexData, EdgeData, TriangleData, Element>::getBoundaryConditions() const
{
	return m_boundaryConditions;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
std::vector<size_t>& Fem<VertexData, EdgeData, TriangleData, Element>::getBoundaryConditions()
{
	return m_boundaryConditions;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
size_t Fem<VertexData, EdgeData, TriangleData, Element>::getBoundaryCondition(size_t id) const
{
	return m_boundaryConditions[id];
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
void Fem<VertexData, EdgeData, TriangleData, Element>::removeBoundaryCondition(size_t id)
{
	m_boundaryConditions.erase(m_boundaryConditions.begin() + id);
}

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM_INL_H
