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

#ifndef SURGSIM_PHYSICS_CONSTRAINTCOMPONENT_H
#define SURGSIM_PHYSICS_CONSTRAINTCOMPONENT_H

#include <memory>
#include <string>

#include "SurgSim/Framework/Component.h"

namespace SurgSim
{
namespace Physics
{

class Constraint;

/// Component class for inserting constraints between physics representations into the scene.
class ConstraintComponent : public SurgSim::Framework::Component
{
public:
	/// Constructor
	/// \param name Name of the component
	explicit ConstraintComponent(const std::string& name);

	/// Destructor
	virtual ~ConstraintComponent();

	/// Set the constraint associated with this component
	/// \param constraint The constraint to be set
	void setConstraint(std::shared_ptr<Constraint> constraint);

	/// Get the constraint associated with this component
	/// \return The associated constraint
	std::shared_ptr<Constraint> getConstraint() const;

protected:
	/// The stored constraint
	std::shared_ptr<Constraint> m_constraint;

	/// Initialize the component
	/// \return true if successful
	bool doInitialize() override;

	/// Wakeup the component
	/// \return true if successful
	bool doWakeUp() override;
};

}; // namespace SurgSim
}; // namespace Physics

#endif // SURGSIM_PHYSICS_CONSTRAINTCOMPONENT_H
