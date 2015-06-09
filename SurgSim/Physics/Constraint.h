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

#ifndef SURGSIM_PHYSICS_CONSTRAINT_H
#define SURGSIM_PHYSICS_CONSTRAINT_H

#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Physics/ConstraintImplementation.h"
#include "SurgSim/Physics/MlcpPhysicsProblem.h"

#include <memory>

namespace SurgSim
{

namespace Physics
{

/// Base class for all physics constraints. Contains data specific to the constraint and a pair of implementations.
class Constraint
{
public:
	/// Sets all the values for this constraints, does validation on the parameters and will throw if something
	/// is wrong with the constraint.
	/// \param constraintType The constraint type.
	/// \param data The data for this constraint.
	/// \param representation0, representation1 Both representations in this constraint.
	/// \param location0, location1 Both locations of the representations involved in this constraint.
	Constraint(
		SurgSim::Physics::ConstraintType constraintType,
		SurgSim::Math::MlcpConstraintType mlcpConstraintType,
		std::shared_ptr<ConstraintData> data,
		std::shared_ptr<Representation> representation0,
		const SurgSim::DataStructures::Location& location0,
		std::shared_ptr<Representation> representation1,
		const SurgSim::DataStructures::Location& location1);

	/// Destructor
	virtual ~Constraint();

	/// Sets all the values for this constraints, does validation on the parameters and will throw if something
	/// is wrong with the constraint.
	/// \param constraintType The constraint type.
	/// \param data The data for this constraint.
	/// \param representation0, representation1 Both representations in this constraint.
	/// \param location0, location1 Both locations of the representations involved in this constraint.
	void setInformation(
		SurgSim::Physics::ConstraintType constraintType,
		SurgSim::Math::MlcpConstraintType mlcpConstraintType,
		std::shared_ptr<ConstraintData> data,
		std::shared_ptr<Representation> representation0,
		const SurgSim::DataStructures::Location& location0,
		std::shared_ptr<Representation> representation1,
		const SurgSim::DataStructures::Location& location1);

	/// Gets both sides implementation as a pair.
	/// \return the pair of implementations forming this constraint.
	const std::pair<std::shared_ptr<ConstraintImplementation>, std::shared_ptr<ConstraintImplementation>>&
		getImplementations() const;

	/// Gets both sides Localization as a pair.
	/// \return the pair of localizations forming this constraint.
	const std::pair<std::shared_ptr<Localization>, std::shared_ptr<Localization>>&
		getLocalizations() const;

	/// Gets the data associated to this constraint.
	/// \return The data associated to this constraint.
	std::shared_ptr<ConstraintData> getData() const;

	/// Gets the number of degree of freedom for this constraint.
	/// \return The number of degree of freedom for this constraint.
	size_t getNumDof() const;

	/// Gets the ConstraintType
	/// \return The type
	SurgSim::Physics::ConstraintType getType();

	/// Gets the ConstraintType for this constraint.
	/// \return	The type.
	SurgSim::Math::MlcpConstraintType getMlcpType();

	/// Builds subset of an Mlcp physics problem associated to this constraint.
	/// \param dt The time step.
	/// \param [in,out] mlcpPhysicsProblem The Mlcp physics problem to be filled up.
	/// \param indexOfRepresentation0 The index of the 1st representation in the Mlcp.
	/// \param indexOfRepresentation1 The index of the 2nd representation in the Mlcp.
	/// \param indexOfConstraint The index of this constraint in the Mlcp.
	void build(double dt,
		MlcpPhysicsProblem* mlcpPhysicsProblem,
		size_t indexOfRepresentation0,
		size_t indexOfRepresentation1,
		size_t indexOfConstraint);

	/// \return Whether this constraint is active.
	bool isActive();

private:
	/// Specific data associated to this constraint
	std::shared_ptr<ConstraintData> m_data;

	/// Pair of implementations defining the 2 sides of the constraint
	std::pair<std::shared_ptr<ConstraintImplementation>, std::shared_ptr<ConstraintImplementation>> m_implementations;
	std::pair<std::shared_ptr<Localization>, std::shared_ptr<Localization>> m_localizations;

	/// The degrees of freedom that this constraint has
	size_t m_numDof;

	/// The type of this constraint
	SurgSim::Physics::ConstraintType m_constraintType;

	/// The type of this mlcp constraint
	SurgSim::Math::MlcpConstraintType m_mlcpConstraintType;

	/// Builds subset of an Mlcp physics problem associated to this constraint user-defined call for extra treatment
	/// \param dt The time step
	/// \param data The data specific to this constraint
	/// \param [in,out] mlcpPhysicsProblem The Mlcp physics problem to be filled up
	/// \param indexOfRepresentation0 The index of the 1st representation in the Mlcp
	/// \param indexOfRepresentation1 The index of the 2nd representation in the Mlcp
	/// \param indexOfConstraint The index of this constraint in the Mlcp
	virtual void doBuild(double dt,
		const ConstraintData& data,
		MlcpPhysicsProblem* mlcpPhysicsProblem,
		size_t indexOfRepresentation0,
		size_t indexOfRepresentation1,
		size_t indexOfConstraint);
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_CONSTRAINT_H
