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

#ifndef SURGSIM_PHYSICS_FEM_H
#define SURGSIM_PHYSICS_FEM_H

#include "SurgSim/DataStructures/MeshElement.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Framework/Asset.h"
#include "SurgSim/Physics/FemElementStructs.h"

namespace SurgSim
{
namespace Physics
{

typedef SurgSim::DataStructures::MeshElement<2, FemElementStructs::FemElement1DParameter> BeamType;
typedef SurgSim::DataStructures::MeshElement<3, FemElementStructs::FemElement2DParameter> TriangleType;
typedef SurgSim::DataStructures::MeshElement<4, FemElementStructs::FemElement3DParameter> TetrahedronType;
typedef SurgSim::DataStructures::MeshElement<8, FemElementStructs::FemElement3DParameter> CubeType;

/// Base class for a data structure for holding FEM mesh data of different dimensions
///
/// Fem itself should not be used directly itself as it contains no overrde for doLoad since the implementation is
/// depedent on the dimension of the FEM you are trying to load. Each dimension overrides the doLoad function present
/// in Asset using its own version of an FemPlyReaderDelegate. Each dimension supports loading both linear and
/// corotational models.
///
/// \tparam VertexData  Type of extra data stored in each vertex
/// \tparam Element		Type of FEM element the mesh will be storing
/// \sa	TriangleMesh
template <class VertexData, class Element>
class Fem : public SurgSim::DataStructures::Vertices<VertexData>, public SurgSim::Framework::Asset,
		public std::enable_shared_from_this<Fem<VertexData, Element>>
{
public:
	/// Default constructor
	Fem();

	/// Adds FEM element to mesh of Element template type
	/// \param element A shared pointer of the Element template type
	/// \return The new size of the vector of elements
	size_t addElement(std::shared_ptr<Element> element);

	/// Gets number of FEM elements in the mesh
	/// \return The number of FEM elements stored
	size_t getNumElements() const;

	/// Gets entire FEM element vector
	/// \return A const vector of all FEM elements stored in the mesh
	const std::vector<std::shared_ptr<Element>>& getElements() const;

	/// Gets entire FEM element vector (non-const)
	/// \return A vector of all FEM elements stored in the mesh
	std::vector<std::shared_ptr<Element>>& getElements();

	/// Retrieve a specific element from the mesh
	/// \param id The id in the element vector
	/// \return A shared pointer to the element
	std::shared_ptr<Element> getElement(size_t id) const;

	/// Add boundary condition to mesh
	/// \param boundaryCondition A vertex id that has a boundary condition
	/// \return The new size of the vector of boundary conditions
	size_t addBoundaryCondition(size_t boundaryCondition);

	/// Gets entire vector of boundary conditions
	/// \return A vector of boundary conditions
	const std::vector<size_t>& getBoundaryConditions() const;

	/// Gets entire vector of boundary conditions (non-const)
	/// \return A vector of boundary conditions
	std::vector<size_t>& getBoundaryConditions();

	/// Retrieves a specific boundary condition
	/// \param id The id of the boundary condition in the vector
	/// \return The vertex id which has a boundary condition
	size_t getBoundaryCondition(size_t id) const;

protected:
	/// Shared loading method for all 3 dimensions
	/// \note Assign template class to the proper dimension PlyReaderDelegate
	template <class PlyType, class FemType>
	bool loadFemFile(const std::string& filename);

	/// Vector of individual elements
	std::vector<std::shared_ptr<Element>> m_elements;

	/// Vector of vertex ids that have boundary conditions
	std::vector<size_t> m_boundaryConditions;
};

} // namespace Physics
} // namespace SurgSim

#include "SurgSim/Physics/Fem-inl.h"

#endif // SURGSIM_PHYSICS_FEM_H
