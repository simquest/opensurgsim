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

namespace Graphics
{
class MeshRepresentation;
class Representation;
}

namespace Physics
{
class DeformableRepresentation;
class Representation;
}

namespace Blocks
{

/// Behavior to copy positions of a Physics Representation to a Graphics Mesh
class TransferPhysicsToGraphicsMeshBehavior : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	/// \param	from	OdeState to get the positions from
	/// \param	to	Vertices to set the positions into
	explicit TransferPhysicsToGraphicsMeshBehavior(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior);

	/// Set the representation from which the positions are from
	/// \param source The physics representation
	void setSource(const std::shared_ptr<SurgSim::Physics::Representation>& source);

	/// Set the representation which will receive the positions
	/// \param target The Graphics Mesh representation
	void setTarget(const std::shared_ptr<SurgSim::Graphics::Representation>& target);

	/// Get the Physics representation which sends the positions
	/// \return The Physics representation which produces positions.
	std::shared_ptr<SurgSim::Physics::Representation> getSource() const;

	/// Get the Graphics representation which receives the positions
	/// \return The Graphics representation which receives positions.
	std::shared_ptr<SurgSim::Graphics::Representation> getTarget() const;

	/// Update the behavior
	/// \param dt	The length of time (seconds) between update calls.
	virtual void update(double dt);

private:
	/// Initialize the behavior
	virtual bool doInitialize() override;

	/// Wakeup the behavior, which simply do a copy (same as update)
	virtual bool doWakeUp() override;

	/// Transfer the data from an OdeState m_from to Vertices m_to
	/// \param doInitialization True if the recipient should be initialized if needed, False otherwise
	/// \note if doInitialization is true and Vertices is empty, it will be filled accordingly
	/// \note with a default vertex data instanciation (if VertexData type is not 'void')
	void transfer(bool doInitialization = false);

	/// Ode state to get the positions from
	std::shared_ptr<SurgSim::Physics::DeformableRepresentation> m_source;

	/// Vertices to set the positions into
	std::shared_ptr<SurgSim::Graphics::MeshRepresentation> m_target;
};

};  // namespace Blocks
};  // namespace SurgSim

#endif  // SURGSIM_BLOCKS_TRANSFERPHYSICSTOGRAPHICSMESHBEHAVIOR_H