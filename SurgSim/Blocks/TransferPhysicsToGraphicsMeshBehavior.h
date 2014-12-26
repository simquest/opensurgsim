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

#ifndef SURGSIM_BLOCKS_TRANSFERPHYSICSTOGRAPHICSMESHBEHAVIOR_H
#define SURGSIM_BLOCKS_TRANSFERPHYSICSTOGRAPHICSMESHBEHAVIOR_H

#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/Macros.h"

namespace SurgSim
{

namespace Framework
{
class Component;
}

namespace Graphics
{
class MeshRepresentation;
}

namespace Physics
{
class DeformableRepresentation;
}

namespace Blocks
{
SURGSIM_STATIC_REGISTRATION(TransferPhysicsToGraphicsMeshBehavior);

/// Behavior to copy positions of a PhysicsRepresentation to a GraphicsMesh.
class TransferPhysicsToGraphicsMeshBehavior : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	explicit TransferPhysicsToGraphicsMeshBehavior(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior);

	/// Set the representation from which the positions are from
	/// \param source The physics representation
	void setSource(const std::shared_ptr<SurgSim::Framework::Component>& source);

	/// Set the representation which will receive the positions
	/// \param target The Graphics Mesh representation
	void setTarget(const std::shared_ptr<SurgSim::Framework::Component>& target);

	/// Get the Physics representation which sends the positions
	/// \return The Physics representation which produces positions.
	std::shared_ptr<SurgSim::Physics::DeformableRepresentation> getSource() const;

	/// Get the Graphics representation which receives the positions
	/// \return The Graphics Mesh representation which receives positions.
	std::shared_ptr<SurgSim::Graphics::MeshRepresentation> getTarget() const;

	void update(double dt) override;

private:
	bool doInitialize() override;
	bool doWakeUp() override;

	/// The DeformableRepresentation from which the Ode state comes.
	std::shared_ptr<SurgSim::Physics::DeformableRepresentation> m_source;

	/// The Graphics Mesh Representation to which the vertices' positions are set.
	std::shared_ptr<SurgSim::Graphics::MeshRepresentation> m_target;
};

};  // namespace Blocks
};  // namespace SurgSim

#endif  // SURGSIM_BLOCKS_TRANSFERPHYSICSTOGRAPHICSMESHBEHAVIOR_H