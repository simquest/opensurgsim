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

#ifndef SURGSIM_PHYSICS_CONSTRAINTIMPLEMENTATION_H
#define SURGSIM_PHYSICS_CONSTRAINTIMPLEMENTATION_H

#include <memory>

#include <Eigen/Core>

#include <SurgSim/Physics/MlcpPhysicsProblem.h>

namespace SurgSim
{

namespace Physics
{

/// Enum defining on which side of the constraint an implementation is (positive or negative side)
/// \note A constraint can be seen as C: side0 = side1 <=> C: side0 - side1 = 0
/// \note Therefore 1 side will have a positive sign (+1), and 1 side a negative sign (-1)
enum ConstraintSideSign {CONSTRAINT_POSITIVE_SIDE, CONSTRAINT_NEGATIVE_SIDE};

class Localization;
class ConstraintData;

/// Base class for all constraint implementations. A ConstraintImplementation defines 1 side of a constraint.
class ConstraintImplementation
{
public:
	/// Default constructor
	ConstraintImplementation();

	/// Constructor
	/// \param localization The localization for this implementation
	/// \note Localization embbed the representation, so it is fully defined
	explicit ConstraintImplementation(std::shared_ptr<Localization> localization);

	/// Destructor
	virtual ~ConstraintImplementation();

	/// Sets the localization associated to this implementation
	/// \param localization The localization for this implementation
	void setLocalization(std::shared_ptr<Localization> localization)
	{
		m_localization = localization;
	}

	/// Gets the localization associated to this implementation
	/// \return The localization of this implementation
	std::shared_ptr<Localization> getLocalization() const
	{
		return m_localization;
	}

	/// Gets the number of degree of freedom for this implementation
	/// \return The number of degree of freedom for this implementation
	unsigned int getNumDof() const
	{
		return doGetNumDof();
	}

	/// Gets the Mixed Linear Complementarity Problem constraint type for this ConstraintImplementation
	/// \return The MLCP constraint type corresponding to this constraint implementation
	SurgSim::Math::MlcpConstraintType getMlcpConstraintType() const
	{
		return doGetMlcpConstraintType();
	}

	/// Builds the subset of an Mlcp physics problem associated to this implementation
	/// \param dt The time step
	/// \param data The data associated to the constraint
	/// \param [in, out] mlcp The Mixed LCP physics problem to fill up
	/// \param indexRepresentation The index, of the representation associated to this implementation. in the mlcp problem
	/// \param indexConstraint The index of the constraint in the mlcp problem
	/// \param sign The sign of this implementation in the constraint (positive or negative side)
	void build(double dt,
		const ConstraintData& data,
		MlcpPhysicsProblem& mlcp,
		unsigned int indexRepresentation,
		unsigned int indexConstraint,
		ConstraintSideSign sign)
	{
		doBuild(dt, data, mlcp, indexRepresentation, indexConstraint, sign);
	}

private:
	/// Localization associated to this implementation
	std::shared_ptr<Localization> m_localization;

	/// Does get number of degree of freedom
	/// \return The number of degree of freedom associated to this implementation
	virtual unsigned int doGetNumDof() const = 0;

	/// Builds the subset of an Mlcp physics problem associated to this implementation
	/// \param dt The time step
	/// \param data The data associated to the constraint
	/// \param [in, out] mlcp The Mixed LCP physics problem to fill up
	/// \param indexRepresentation The index, of the representation associated to this implementation. in the mlcp problem
	/// \param indexConstraint The index of the constraint in the mlcp problem
	/// \param sign The sign of this implementation in the constraint (positive or negative side)
	virtual void doBuild(double dt,
				const ConstraintData& data,
				MlcpPhysicsProblem& mlcp,
				unsigned int indexRepresentation,
				unsigned int indexConstraint,
				ConstraintSideSign sign) = 0;

	/// Gets the Mixed Linear Complementarity Problem constraint type for this ConstraintImplementation
	/// \return The MLCP constraint type corresponding to this constraint implementation
	virtual SurgSim::Math::MlcpConstraintType doGetMlcpConstraintType() const
	{
		return SurgSim::Math::MLCP_INVALID_CONSTRAINT;
	}
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_CONSTRAINTIMPLEMENTATION_H
