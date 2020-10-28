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

#ifndef SURGSIM_BLOCKS_DRIVEELEMENTFROMINPUTBEHAVIOR_H
#define SURGSIM_BLOCKS_DRIVEELEMENTFROMINPUTBEHAVIOR_H

#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/ObjectFactory.h"

namespace SurgSim
{

namespace Input
{
class InputComponent;
}

namespace Blocks
{

SURGSIM_STATIC_REGISTRATION(DriveElementFromInputBehavior);

/// Behavior to copy a pose from an input component to a SceneElement
/// By adding this behavior to a SceneElement, that SceneElement will be moved
/// in correspondance to the input.
/// Defaults to updating in the PhysicsManager loop. An instance could run in the BehaviorManager if the SceneElement
/// does not have any physics or collision representations.
class DriveElementFromInputBehavior : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	explicit DriveElementFromInputBehavior(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Blocks::DriveElementFromInputBehavior);

	int getTargetManagerType() const override;

	/// Set the InputComponent that provides the pose
	/// \param	source A SurgSim::Input::InputComponent
	void setSource(std::shared_ptr<SurgSim::Framework::Component> source);

	/// Get the InputComponent which is being used by this behavior
	/// \return A SurgSim::Component::InputComponent
	std::shared_ptr<SurgSim::Framework::Component> getSource();

	/// Set name of the pose.
	/// \param	poseName	The name of the pose.
	void setPoseName(const std::string& poseName);

	/// Get name of the pose.
	/// \return The name of the pose.
	std::string getPoseName();

	/// Update the behavior
	/// \param dt	The length of time (seconds) between update calls.
	virtual void update(double dt);

protected:
	/// Initialize the behavior
	virtual bool doInitialize();

	/// Wakeup the behavior, which copies the initial pose
	virtual bool doWakeUp();

private:
	/// InputComponent to get the pose
	std::shared_ptr<SurgSim::Input::InputComponent> m_source;

	std::string m_poseName;
};


};  // namespace Blocks

};  // namespace SurgSim


#endif // SURGSIM_BLOCKS_DRIVEELEMENTFROMINPUTBEHAVIOR_H
