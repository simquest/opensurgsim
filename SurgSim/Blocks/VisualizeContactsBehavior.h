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

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Framework/Behavior.h"

namespace SurgSim
{

namespace DataStructures
{
template<class T>
class SafeReadAccessor;
}

namespace Graphics
{
class VectorFieldRepresentation;
}

namespace Blocks
{

/// This behavior is used to visualize the contacts
/// on collision representation through vector field
class VisualizeContactsBehavior: public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	explicit VisualizeContactsBehavior(const std::string& name);

	/// Set the collision representation whose contacts, if any, will be visualized.
	/// \param	collisionRepresentation The collision representation of a stapler.
	void setCollisionRepresentation(std::shared_ptr<SurgSim::Collision::Representation> collisionRepresentation);

	/// Update the behavior, show vector field for contacts if there is any.
	/// \param dt	The length of time (seconds) between update calls.
	virtual void update(double dt) override;

	/// Return the type of manager that should be responsible for this behavior
	/// \return An integer indicating which manger should be responsible for this behavior.
	virtual int getTargetManagerType() const override;

	/// Set the scale of vector field, default 1.0.
	// \param scale The scale of the vector field.
	void setVectorFieldScale(double scale);

protected:
	/// Initialize this behavior
	/// \return True on success, otherwise false.
	/// \note In current implementation, this method always returns "true".
	virtual bool doInitialize() override;

	/// Wakeup this behavior
	/// \return True on success, otherwise false.
	/// \note In current implementation, this method always returns "true".
	virtual bool doWakeUp() override;

private:
	/// The collision representation to get contacts for visualizing.
	std::shared_ptr<SurgSim::Collision::Representation> m_collisionRepresentation;

	/// The collision map for the representation.
	std::unique_ptr<SurgSim::DataStructures::SafeReadAccessor<SurgSim::Collision::ContactMapType>> m_collisions;

	/// The osg vector field for visualizing contacts on collision representation
	std::shared_ptr<SurgSim::Graphics::VectorFieldRepresentation> m_vectorField;
};

} // namesapce Blocks
} // namespace SurgSim

#endif  // SURGSIM_BLOCKS_VISUALIZECONTACTSBEHAVIOR_H
