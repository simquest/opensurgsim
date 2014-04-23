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

#ifndef SURGSIM_BLOCKS_DRIVEELEMENTBEHAVIOR_H
#define SURGSIM_BLOCKS_DRIVEELEMENTBEHAVIOR_H

#include "SurgSim/Framework/Behavior.h"


namespace SurgSim
{

namespace Physics
{
class Representation;
}

namespace Blocks
{

/// Behavior to drive the SceneElement pose with a Physics Representation
/// By adding this behavior to a SceneElement, that SceneElement will be moved
/// in correspondance to the source PhysicsRepresentation.
class DriveElementBehavior : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	explicit DriveElementBehavior(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Blocks::DriveElementBehavior);

	/// Set the component which drives the SceneElement
	/// \param source A SurgSim::Physics::Representation
	void setSource(std::shared_ptr<SurgSim::Framework::Component> source);

	/// Get the component which drives the SceneElement
	/// \return A SurgSim::Physics::Representation
	std::shared_ptr<SurgSim::Framework::Component> getSource();

	/// Update the behavior
	/// \param dt	The length of time (seconds) between update calls.
	virtual void update(double dt);

protected:
	/// Initialize the behavior
	virtual bool doInitialize();

	/// Wakeup the behavior, which copies the initial pose
	virtual bool doWakeUp();

private:
	/// Representation to drive the scene element
	std::shared_ptr<SurgSim::Physics::Representation> m_source;
};


};  // namespace Blocks

};  // namespace SurgSim


#endif // SURGSIM_BLOCKS_DRIVEELEMENTBEHAVIOR_H
