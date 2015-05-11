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

#ifndef SURGSIM_PHYSICS_FEMELEMENTMESH_INL_H
#define SURGSIM_PHYSICS_FEMELEMENTMESH_INL_H

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/Log.h"

using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Physics
{

template <class VertexData, class EdgeData, class TriangleData, class Element>
FemElementMesh<VertexData, EdgeData, TriangleData, Element>::FemElementMesh()
{
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
size_t FemElementMesh<VertexData, EdgeData, TriangleData, Element>::addFemElement(const Element& element)
{
	m_femElements.push_back(element);
	return m_femElements.size() - 1;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
size_t FemElementMesh<VertexData, EdgeData, TriangleData, Element>::getNumElements() const
{
	return m_femElements.size();
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
const std::vector<Element>& FemElementMesh<VertexData, EdgeData, TriangleData, Element>::getFemElements() const
{
	return m_femElements;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
std::vector<Element>& FemElementMesh<VertexData, EdgeData, TriangleData, Element>::getFemElements()
{
	return m_femElements;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
const Element& FemElementMesh<VertexData, EdgeData, TriangleData, Element>::getFemElement(size_t id) const
{
	return m_femElements[id];
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
void FemElementMesh<VertexData, EdgeData, TriangleData, Element>::removeFemElement(size_t id)
{
	m_femElements.erase(m_femElements.begin() + id);
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
size_t FemElementMesh<VertexData, EdgeData, TriangleData, Element>::addBoundaryCondition(const size_t boundaryCondition)
{
	m_boundaryConditions.push_back(boundaryCondition);
	return m_boundaryConditions.size() - 1;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
const std::vector<size_t>& FemElementMesh<VertexData, EdgeData, TriangleData, Element>::getBoundaryConditions() const
{
	return m_boundaryConditions;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
std::vector<size_t>& FemElementMesh<VertexData, EdgeData, TriangleData, Element>::getBoundaryConditions()
{
	return m_boundaryConditions;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
const size_t FemElementMesh<VertexData, EdgeData, TriangleData, Element>::getBoundaryCondition(size_t id) const
{
	return m_boundaryConditions[id];
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
void FemElementMesh<VertexData, EdgeData, TriangleData, Element>::removeBoundaryCondition(size_t id)
{
	m_boundaryConditions.erase(m_boundaryConditions.begin() + id);
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
double FemElementMesh<VertexData, EdgeData, TriangleData, Element>::getYoungModulus() const
{
	return m_youngModulus;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
double FemElementMesh<VertexData, EdgeData, TriangleData, Element>::getPoissonRatio() const
{
	return m_poissonRatio;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
double FemElementMesh<VertexData, EdgeData, TriangleData, Element>::getMassDensity() const
{
	return m_massDensity;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
void FemElementMesh<VertexData, EdgeData, TriangleData, Element>::setYoungModulus(double modulus)
{
	m_youngModulus = modulus;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
void FemElementMesh<VertexData, EdgeData, TriangleData, Element>::setPoissonRatio(double ratio)
{
	m_poissonRatio = ratio;
}

template <class VertexData, class EdgeData, class TriangleData, class Element>
void FemElementMesh<VertexData, EdgeData, TriangleData, Element>::setMassDensity(double density)
{
	m_massDensity = density;
}

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEMELEMENTMESH_INL_H
