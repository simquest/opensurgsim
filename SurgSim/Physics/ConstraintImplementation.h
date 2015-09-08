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

#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Physics/ConstraintImplementationFactory.h"
#include "SurgSim/Physics/ConstraintType.h"
#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/MlcpPhysicsProblem.h"

namespace SurgSim
{

namespace Physics
{

/// Enum defining on which side of the constraint an implementation is (positive or negative side)
/// \note A constraint can be seen as C: side0 = side1 <=> C: side0 - side1 = 0
/// \note Therefore 1 side will have a positive sign (+1), and 1 side a negative sign (-1)
enum ConstraintSideSign {CONSTRAINT_POSITIVE_SIDE, CONSTRAINT_NEGATIVE_SIDE};

/// Base class for all constraint implementations. A ConstraintImplementation defines 1 side of a constraint.
class ConstraintImplementation
{
public:
	/// Constructor
	ConstraintImplementation();

	/// Destructor
	virtual ~ConstraintImplementation();

	/// \return The static class factory that contains the implementations for a given Representation type.
	static ConstraintImplementationFactory& getFactory();

	/// Gets the number of degree of freedom for this implementation
	/// \return The number of degree of freedom for this implementation
	size_t getNumDof() const;

	/// Gets the constraint type for this ConstraintImplementation
	/// \return The constraint type corresponding to this constraint implementation
	virtual SurgSim::Physics::ConstraintType getConstraintType() const = 0;

	/// Builds the subset of an Mlcp physics problem associated to this implementation
	/// \param dt The time step
	/// \param data The data associated to the constraint
	/// \param localization The localization for this implementation
	/// \param [in, out] mlcp The Mixed LCP physics problem to fill up
	/// \param indexOfRepresentation The index of the representation (associated to this implementation) in the mlcp
	/// \param indexOfConstraint The index of the constraint in the mlcp
	/// \param sign The sign of this implementation in the constraint (positive or negative side)
	void build(double dt,
		const ConstraintData& data,
		const std::shared_ptr<Localization>& localization,
		MlcpPhysicsProblem* mlcp,
		size_t indexOfRepresentation,
		size_t indexOfConstraint,
		ConstraintSideSign sign);

protected:
	/// Preallocated variable for derived implementations of doBuild.
	Eigen::SparseVector<double, Eigen::RowMajor, ptrdiff_t> m_newH;

private:

	/// Does get number of degree of freedom
	/// \return The number of degree of freedom associated to this implementation
	virtual size_t doGetNumDof() const = 0;

	/// Builds the subset of an Mlcp physics problem associated to this implementation
	/// \param dt The time step
	/// \param data The data associated to the constraint
	/// \param localization The localization for the constraint
	/// \param [in, out] mlcp The Mixed LCP physics problem to fill up
	/// \param indexOfRepresentation The index of the representation (associated to this implementation) in the mlcp
	/// \param indexOfConstraint The index of the constraint in the mlcp
	/// \param sign The sign of this implementation in the constraint (positive or negative side)
	virtual void doBuild(double dt,
				const ConstraintData& data,
				const std::shared_ptr<Localization>& localization,
				MlcpPhysicsProblem* mlcp,
				size_t indexOfRepresentation,
				size_t indexOfConstraint,
				ConstraintSideSign sign) = 0;

};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_CONSTRAINTIMPLEMENTATION_H
