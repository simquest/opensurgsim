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

#ifndef SURGSIM_BLOCKS_VISUALIZECONTACTSBEHAVIOR_H
#define SURGSIM_BLOCKS_VISUALIZECONTACTSBEHAVIOR_H

#include <memory>
#include <string>

#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/ObjectFactory.h"

namespace SurgSim
{

namespace Graphics
{
class VectorFieldRepresentation;
}

namespace Collision
{
class Representation;
}

namespace Blocks
{

SURGSIM_STATIC_REGISTRATION(VisualizeContactsBehavior);

/// This behavior is used to visualize the contacts
/// on collision representation through vector field
class VisualizeContactsBehavior: public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	explicit VisualizeContactsBehavior(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Blocks::VisualizeContactsBehavior);

	/// Used for serialization.
	/// \return The collision representation whose contacts will be visualized.
	std::shared_ptr<SurgSim::Framework::Component> getCollisionRepresentation();

	/// Used for serialization.
	/// \param	collisionRepresentation The collision representation whose contacts will be visualized.
	void setCollisionRepresentation(std::shared_ptr<SurgSim::Framework::Component> collisionRepresentation);

	/// Update the behavior, show vector field for contacts if there is any.
	/// \param dt	The length of time (seconds) between update calls.
	void update(double dt) override;

	/// Return the type of manager that should be responsible for this behavior
	/// \return An integer indicating which manger should be responsible for this behavior.
	int getTargetManagerType() const override;

	/// \return The scale of the vector field.
	double getVectorFieldScale();

	/// Set the scale of vector field, default 1.0.
	// \param scale The scale of the vector field.
	void setVectorFieldScale(double scale);

protected:
	/// Initialize this behavior
	/// \return True on success, otherwise false.
	/// \note In current implementation, this method always returns "true".
	bool doInitialize() override;

	/// Wakeup this behavior
	/// \return True on success, otherwise false.
	/// \note In current implementation, this method always returns "true".
	bool doWakeUp() override;

private:
	/// The collision representation to get contacts for visualizing.
	std::shared_ptr<SurgSim::Collision::Representation> m_collisionRepresentation;

	/// The osg vector field for visualizing contacts on collision representation
	std::shared_ptr<SurgSim::Graphics::VectorFieldRepresentation> m_vectorField;
};

} // namespace Blocks
} // namespace SurgSim

#endif  // SURGSIM_BLOCKS_VISUALIZECONTACTSBEHAVIOR_H
