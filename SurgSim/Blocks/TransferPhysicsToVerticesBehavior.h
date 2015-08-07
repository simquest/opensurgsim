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

#ifndef SURGSIM_BLOCKS_TRANSFERPHYSICSTOVERTICESEHAVIOR_H
#define SURGSIM_BLOCKS_TRANSFERPHYSICSTOVERTICESEHAVIOR_H

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Framework/Behavior.h"

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

/// Transfer Physics State to any representation that has a Vertices property, the "Vertices" property on the targets
/// side needs to accept DataStructures::VerticesPlain for the type.
class TransferPhysicsToVerticesBehavior : public Framework::Behavior
{
public:

	/// Constructor
	/// \param name Name of the behavior
	explicit TransferPhysicsToVerticesBehavior(const std::string& name);

	/// \return the source representation of this behavior
	std::shared_ptr<Physics::DeformableRepresentation> getSource() const;

	/// Sets the source for this behavior, all the positions from the state of the source
	/// will be copied into a Vertices object and set in the target, using setValue("Vertices")
	/// \param source the component that is the source
	void setSource(const std::shared_ptr<Framework::Component>& source);

	/// \return the target representation of this behavior
	std::shared_ptr<Framework::Component> getTarget() const;

	/// Sets the target for this behavior, it needs to have a "Vertices" property that takes
	/// DataStructures::VerticesPlain as a parameter
	/// \throws if target does not have a "Vertices" property
	/// \param target the representation to be used as a target
	void setTarget(const std::shared_ptr<Framework::Component>& target);

	/// override update
	/// \throws if the type of the "Vertices" property on the target is not DataStructures::VerticesPlain
	void update(double dt) override;

	bool doInitialize() override;

	bool doWakeUp() override;

private:

	std::shared_ptr<Physics::DeformableRepresentation> m_source; ///@< Source representation
	std::shared_ptr<Framework::Component> m_target; ///@< Target representation

	/// vertices structure that is used for the update
	DataStructures::VerticesPlain m_vertices;
};
}
}


#endif
