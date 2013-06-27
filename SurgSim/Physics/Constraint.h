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

#include <SurgSim/Framework/Log.h>

#include <SurgSim/Physics/ConstraintData.h>
#include <SurgSim/Physics/ConstraintImplementation.h>
#include <SurgSim/Physics/MlcpPhysicsProblem.h>

#include <Eigen/Core>

#include <utility>
#include <memory>

namespace SurgSim
{

namespace Physics
{

/// Base class for all physics constraints. Contains data specific to the constraint and a pair of implementations.
class Constraint
{
public:
	/// Default constructor
	Constraint();
	
	/// Constructor
	/// \param side0, side1 Both sides implementation of the constraint
	Constraint(std::shared_ptr<ConstraintImplementation> side0, std::shared_ptr<ConstraintImplementation> side1);
	
	/// Destructor
	virtual ~Constraint();

	/// Sets both sides implementation
	/// \param side0, side1 Both sides implementation of the constraint
	void setImplementations(std::shared_ptr<ConstraintImplementation> side0, std::shared_ptr<ConstraintImplementation> side1)
	{
		m_implementations = std::make_pair(side0, side1);
	}
	
	/// Gets both sides ConstraintImplementation as a pair
	/// \return the pair of ConstraintImplementation forming this constraint
	const std::pair<std::shared_ptr<ConstraintImplementation>, std::shared_ptr<ConstraintImplementation>>& getImplementations() const
	{
		return m_implementations;
	}

	/// Sets the data associated to this constraint
	/// \param data The data for this constraint
	void setData(std::shared_ptr<ConstraintData> data)
	{
		m_data = data;
	}

	/// Gets the data associated to this constraint
	/// \return The data associated to this constraint
	std::shared_ptr<ConstraintData> getData()
	{
		return m_data;
	}

	/// Builds subset of an Mlcp physics problem associated to this constraint
	/// \param dt The time step
	/// \param [in, out] mclpPhysicsProblem The Mlcp physics problem to be filled up
	/// \param indexActor0 The index of the 1st representation in the Mlcp
	/// \param indexActor1 The index of the 2nd representation in the Mlcp
	/// \param indexConstraing The index of this constraint in the Mlcp
	void build(double dt,
		MlcpPhysicsProblem& mlcpPhysicsProblem,
		unsigned int indexRepresentation0,
		unsigned int indexRepresentation1,
		unsigned int indexConstraint)
	{
		doBuild(dt, *m_data, mlcpPhysicsProblem, indexRepresentation0, indexRepresentation1, indexConstraint);
	}

	/// Gets the number of degree of freedom for this constraint
	/// \return The number of degree of freedom for this constraint
	unsigned int getNumDof()
	{
		using namespace SurgSim::Framework;

		// TODO: Assert that both sides have same DOF
		SURGSIM_ASSERT(m_implementations.first->getNumDof() == m_implementations.second->getNumDof()) << 
			"Both sides of the constraint should have the same number of Dof ("<< m_implementations.first->getNumDof() <<
			" != " << m_implementations.second->getNumDof() <<")" << std::endl;
		return m_implementations.first->getNumDof();
	}

	/// Clears/resets this constraint
	void clear()
	{
		doClear();
		m_implementations.first = nullptr;
		m_implementations.second = nullptr;
		m_data = nullptr;
	}

private:
	/// Specific data associated to this constraint
	std::shared_ptr<ConstraintData> m_data;
	/// Pair of implementations defining the 2 sides of the constraint
	std::pair<std::shared_ptr<ConstraintImplementation>, std::shared_ptr<ConstraintImplementation>> m_implementations;

	/// Builds subset of an Mlcp physics problem associated to this constraint
	/// \param dt The time step
	/// \param data The data specific to this constraint
	/// \param [in, out] mclpPhysicsProblem The Mlcp physics problem to be filled up
	/// \param indexActor0 The index of the 1st representation in the Mlcp
	/// \param indexActor1 The index of the 2nd representation in the Mlcp
	/// \param indexConstraing The index of this constraint in the Mlcp
	virtual void doBuild(double dt,
		const ConstraintData& data,
		MlcpPhysicsProblem& mlcpPhysicsProblem,
		unsigned int offsetActor0,
		unsigned int offsetActor1,
		unsigned int offsetConstraint);

	/// Clears/resets user-defined call for extra treatment
	virtual void doClear();
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_CONSTRAINT_H
