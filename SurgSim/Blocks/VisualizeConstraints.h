// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_BLOCKS_VISUALIZECONSTRAINTS_H
#define SURGSIM_BLOCKS_VISUALIZECONSTRAINTS_H

#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Physics/PhysicsManagerState.h"

namespace SurgSim
{

namespace Physics
{
class PhysicsManager;
}

namespace Graphics
{
class VectorFieldRepresentation;
}

namespace Blocks
{
SURGSIM_STATIC_REGISTRATION(VisualizeConstraintsBehavior);


/// Behavior to visualize information about the constraints as they are in the physics manager
/// this will show the constraint location and the force from the last iteration of the physics manager
/// \note currently only two types of constraint are being visualized MLCP_BILATERAL_3D_CONSTRAINT and
/// MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT
class VisualizeConstraintsBehavior : public SurgSim::Framework::Behavior
{
public:
	explicit VisualizeConstraintsBehavior(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Blocks::VisualizeConstraintsBehavior);

	/// Set the graphics Vectorfield to be used to display the constraint of the given physics manager group
	/// \param constraintType internal group type e.g. CONSTRAINT_GROUP_TYPE_CONTACT
	/// \param vectorField vectorField to be used for visualization of this constraint group
	void setVectorField(Physics::ConstraintGroupType constraintType,
						const std::shared_ptr<Framework::Component>& vectorField);

	typedef std::vector<std::pair<int, std::shared_ptr<Framework::Component>>> FieldsType;

	/// Sets all of the fields in one swoop
	/// \param fields list of all the fields and their constraint group
	void setVectorFields(const FieldsType& fields);

	/// \return all the vector fields used in this behavior
	FieldsType getVectorFields() const;

	void update(double dt) override;

	int getTargetManagerType() const  override;

private:


	bool doInitialize() override;

	bool doWakeUp() override;

	/// Need reference to physics manager for introspection into the state
	std::weak_ptr<SurgSim::Physics::PhysicsManager> m_manager;

	std::map<SurgSim::Physics::ConstraintGroupType, std::shared_ptr<SurgSim::Graphics::VectorFieldRepresentation>>
			m_graphics;

	boost::mutex m_graphicsMutex;
};

/// SceneElement that generates the VisualizeConstraintBehavior and the appropriate graphics Vectorfield
class VisualizeConstraints : public SurgSim::Framework::BasicSceneElement
{
public:
	explicit VisualizeConstraints(const std::string& name = "ConstraintVisualization");
};

}; // namespace Blocks

}; // namespace SurgSim

#endif // SURGSIM_BLOCKS_VISUALIZECONSTRAINTS_H
