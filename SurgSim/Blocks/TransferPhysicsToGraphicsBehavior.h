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

#ifndef SURGSIM_BLOCKS_TRANSFERPHYSICSTOGRAPHICSBEHAVIOR_H
#define SURGSIM_BLOCKS_TRANSFERPHYSICSTOGRAPHICSBEHAVIOR_H

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
class Representation;
}

namespace Physics
{
class DeformableRepresentation;
}

namespace Blocks
{
SURGSIM_STATIC_REGISTRATION(TransferPhysicsToGraphicsBehavior);

/// Behavior to copy positions of a PhysicsRepresentation to a Graphics Mesh or PointCloud.
/// \note Currently only supports the transfer to Graphics::MeshRepresentation and Graphics::PointCloudRepresentation
class TransferPhysicsToGraphicsBehavior : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	explicit TransferPhysicsToGraphicsBehavior(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Blocks::TransferPhysicsToGraphicsBehavior);

	/// Set the representation from which the positions are from
	/// \param source The physics representation
	void setSource(const std::shared_ptr<SurgSim::Framework::Component>& source);

	/// Set the representation which will receive the positions
	/// \param target The Graphics Mesh representation
	void setTarget(const std::shared_ptr<SurgSim::Framework::Component>& target);

	/// Get the Physics representation which sends the positions
	/// \return The Physics representation which produces positions.
	std::shared_ptr<SurgSim::Framework::Component> getSource() const;

	/// Get the Graphics representation which receives the positions
	/// \return The Graphics representation which receives positions.
	std::shared_ptr<SurgSim::Framework::Component> getTarget() const;

	virtual void update(double dt) override;

private:
	virtual bool doInitialize() override;
	virtual bool doWakeUp() override;

	/// The DeformableRepresentation from which the Ode state comes.
	std::shared_ptr<SurgSim::Physics::DeformableRepresentation> m_source;

	/// The Graphics Representation to which the vertices' positions are set.
	std::shared_ptr<SurgSim::Graphics::Representation> m_target;
};

};  // namespace Blocks
};  // namespace SurgSim

#endif  // SURGSIM_BLOCKS_TRANSFERPHYSICSTOGRAPHICSBEHAVIOR_H