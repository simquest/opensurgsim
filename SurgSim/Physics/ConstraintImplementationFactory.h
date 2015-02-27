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

#ifndef SURGSIM_PHYSICS_CONSTRAINTIMPLEMENTATIONFACTORY_H
#define SURGSIM_PHYSICS_CONSTRAINTIMPLEMENTATIONFACTORY_H

#include <array>
#include <memory>
#include <typeindex>
#include <unordered_map>

#include "SurgSim/Math/MlcpConstraintType.h"
#include "SurgSim/Physics/Representation.h"

namespace SurgSim
{
namespace Physics
{

class ConstraintImplementation;

/// This class manages ConstraintImplementations, and can be used to look up the correct implementation
/// by representation and constraint type.
/// The only maintenance that needs to be done right now when a new
/// ConstraintImplementation is added is to add a call into the constructor.
class ConstraintImplementationFactory
{
public:

	/// Constructor
	ConstraintImplementationFactory();

	/// Destructor
	~ConstraintImplementationFactory();

	/// Get the instance of a ConstraintImplementation for a specific representation and
	/// constraint type.
	/// \param	representationType	Type index of the representation.
	/// \param	constraintType	  	Type of the constraint.
	/// \return	a pointer to an implementation if the implementation can be found, nullptr otherwise.
	std::shared_ptr<ConstraintImplementation> getImplementation(
		std::type_index representationType,
		SurgSim::Math::MlcpConstraintType constraintType);

	/// Add an implementation to the internal directory.
	/// \param typeIndex The type of representation associated with the implementation.
	/// \param	implementation	The ConstraintImplementation to add.
	void addImplementation(std::type_index typeIndex, std::shared_ptr<ConstraintImplementation> implementation);

private:

	/// Lookup table for constraint implementations
	std::unordered_map<std::type_index,
		std::array<std::shared_ptr<ConstraintImplementation>, SurgSim::Math::MLCP_NUM_CONSTRAINT_TYPES>>
		m_implementations;
};

}; // Physics
}; // SurgSim

#endif