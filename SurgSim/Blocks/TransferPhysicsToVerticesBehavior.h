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

#ifndef SURGSIM_BLOCKS_TRANSFERPHYSICSTOCURVEBEHAVIOR_H
#define SURGSIM_BLOCKS_TRANSFERPHYSICSTOCURVEBEHAVIOR_H

#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/DataStructures/EmptyData.h"

namespace SurgSim
{

namespace Physics
{
class DeformableRepresentation;
}
namespace Graphics
{
class CurveRepresentation;
}

namespace Blocks
{

class TransferPhysicsToVerticesBehavior : public SurgSim::Framework::Behavior
{
public:
	TransferPhysicsToVerticesBehavior(const std::string& name);

	std::shared_ptr<SurgSim::Physics::DeformableRepresentation> getSource() const;
	void setSource(const std::shared_ptr<SurgSim::Framework::Component>& source);

	std::shared_ptr<SurgSim::Framework::Component> getTarget() const;
	void setTarget(const std::shared_ptr<SurgSim::Framework::Component>& target);

	virtual void update(double dt) override;

	virtual bool doInitialize() override;

	virtual bool doWakeUp() override;

private:

	std::shared_ptr<SurgSim::Physics::DeformableRepresentation> m_source;
	std::shared_ptr<SurgSim::Framework::Component> m_target;

	SurgSim::DataStructures::Vertices<SurgSim::DataStructures::EmptyData> m_vertices;
};
}
}


#endif
