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

#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "../Framework/BasicSceneElement.h"

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

class VisualizeConstraintsBehavior : public SurgSim::Framework::Behavior
{
public:
	explicit VisualizeConstraintsBehavior(const std::string& name);

	void setVectorField(SurgSim::Physics::ConstraintGroupType constraintType,
						std::shared_ptr<SurgSim::Graphics::VectorFieldRepresentation> vectorField);

	void update(double dt) override;

	/// Specifies which manger will handle this behavior
	virtual int getTargetManagerType() const;

private:
	bool doInitialize() override;

	bool doWakeUp() override;

	std::weak_ptr<SurgSim::Physics::PhysicsManager> m_manager;
	std::map<SurgSim::Physics::ConstraintGroupType, std::shared_ptr<SurgSim::Graphics::VectorFieldRepresentation>>
			m_gfxVectorField;
};

class VisualizeConstraints : public SurgSim::Framework::BasicSceneElement
{
public:
	explicit VisualizeConstraints(const std::string& name = "ConstraintVisualization");
};

}; // namespace Blocks

}; // namespace SurgSim

#endif // SURGSIM_BLOCKS_VISUALIZECONSTRAINTS_H
