// This file is a part of the OpenSurgSim project.
// Copyright 2020, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PHYSICS_MASSSPRINGMODEL_H
#define SURGSIM_PHYSICS_MASSSPRINGMODEL_H

#include <vector>

#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Framework/Asset.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/Mass.h"

namespace SurgSim
{
namespace Physics
{

class Spring;

SURGSIM_STATIC_REGISTRATION(MassSpringModel);

/// Base class for a data structure for holding MassSpring mesh data.
class MassSpringModel : public SurgSim::DataStructures::Vertices<Mass>, public SurgSim::Framework::Asset,
	public std::enable_shared_from_this<MassSpringModel>
{
public:
	/// Default constructor
	MassSpringModel();

	SURGSIM_CLASSNAME(SurgSim::Physics::MassSpringModel);

	/// Adds a mass
	/// \param mass The mass to add to the representation
	/// \note Masses are kept in an ordered list, giving them an index
	/// \note This mass will be associated with the node of same index in any associated OdeState
	void addMass(const std::shared_ptr<Mass> mass);

	/// Adds a spring, and sets its rest length.
	/// \param spring The spring to add to the representation
	void addSpring(const std::shared_ptr<Spring> spring);

	/// Gets the number of masses
	/// \return the number of masses
	size_t getNumMasses() const;

	/// Gets the number of springs
	/// \return the number of springs
	size_t getNumSprings() const;

	/// Retrieves the masses.
	const std::vector<std::shared_ptr<Mass>>& getMasses() const;

	/// Retrieves a given spring from its id
	/// \param springId The spring id for which the spring is requested
	/// \return the spring for the given springId
	/// \note The spring is returned with read/write access
	/// \note Out of range springId will raise an exception
	const std::vector<std::shared_ptr<Spring>>& getSprings() const;

	/// Retrieves the mass of a given node
	/// \param nodeId The node id for which the mass is requested
	/// \return the mass attribute of a node
	/// \note Out of range nodeId will raise an exception
	const std::shared_ptr<Mass>& getMass(size_t nodeId) const;

	/// Retrieves a given spring from its id
	/// \param springId The spring id for which the spring is requested
	/// \return the spring for the given springId
	/// \note Out of range springId will raise an exception
	const std::shared_ptr<Spring>& getSpring(size_t springId) const;

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

	/// Stores an element's node ids, e.g., for a 2d shape this may store triangle nodes.
	/// \param ids The vector of node ids (i.e., mass ids) for the new element.
	/// \return The new size of the vector of elements.
	size_t addElement(const std::vector<size_t>& nodeIds);

	/// Get an element's node ids.
	/// \param index The index of the element.
	/// \return The vector of node ids.
	const std::vector<size_t>& getNodeIds(size_t index) const;

	/// \return The number of elements.
	size_t getNumElements() const;

	/// \return The number of nodes per element.
	size_t getNumNodesPerElement() const;

	/// \param The radius to set.
	void setRadius(double radius);
	/// \return The radius.
	const DataStructures::OptionalValue<double>& getRadius() const;

	/// \param The thickness to set.
	void setThickness(double thickness);
	/// \return The thickness.
	const DataStructures::OptionalValue<double>& getThickness() const;

	/// Save the current MassSpring mesh to a ply file. See MassSpringPlyReaderDelegate for the file structure.
	/// \param fileName Name of the file for writing.
	/// \param physicsLength The radius or thickness (for 2 or 3-node elements), if not already in the MassSpringModel.
	/// \return true if the file was written successfully.
	bool save(const std::string& fileName, double physicsLength = 0.0) const;

protected:
	bool doLoad(const std::string& filePath) override;

	/// Masses
	std::vector<std::shared_ptr<Mass>> m_masses;

	/// Springs
	std::vector<std::shared_ptr<Spring>> m_springs;

	/// Vector of vertex ids that have boundary conditions
	std::vector<size_t> m_boundaryConditions;

	/// The node ids for each element, e.g., for triangles it contains the three node ids for each triangle.
	std::vector<std::vector<size_t>> m_nodeIds;

	/// The radius, if any.
	DataStructures::OptionalValue<double> m_radius;

	/// The thickness, if any.
	DataStructures::OptionalValue<double> m_thickness;
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_MASSSPRINGMODEL_H
