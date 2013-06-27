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

#ifndef SURGSIM_PHYSICS_FIXEDREPRESENTATIONCONTACT_H
#define SURGSIM_PHYSICS_FIXEDREPRESENTATIONCONTACT_H

#include <SurgSim/Physics/ConstraintImplementation.h>

namespace SurgSim
{

namespace Physics
{

/// FixedRepresentation frictionless contact implementation
class FixedRepresentationContact : public ConstraintImplementation
{
public:
	/// Constructor
	/// \param localization The localization of the contact on the FixedRepresentation
	explicit FixedRepresentationContact(std::shared_ptr<Localization> localization) :
	ConstraintImplementation(localization)
	{}

	/// Destructor
	virtual ~FixedRepresentationContact()
	{}

private:
	/// Gets the number of degree of freedom
	/// \return 1 as a frictionless contact is formed of 1 equation of constraint (along the normal direction)
	unsigned int doGetNumDof() const override
	{
		return 1;
	}

	/// Builds the subset of an Mlcp physics problem associated to this implementation
	/// \param dt The time step
	/// \param data The data associated to the constraint
	/// \param [in, out] mlcp The Mixed LCP physics problem to fill up
	/// \param indexRepresentation The index, of the representation associated to this implementation. in the mlcp problem
	/// \param indexConstraint The index of the constraint in the mlcp problem
	/// \param sign The sign of this implementation in the constraint (positive or negative side)
	/// \note Empty for a Fixed Representation
	void doBuild(double dt,
		const ConstraintData& data,
		MlcpPhysicsProblem& mlcp,
		unsigned int indexRepresentation,
		unsigned int indexConstraint,
		ConstraintSideSign sign) override
	{
	}
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_FIXEDREPRESENTATIONCONTACT_H
