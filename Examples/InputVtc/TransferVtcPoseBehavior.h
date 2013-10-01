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

#ifndef EXAMPLES_INPUTVTC_TRANSFERVTCPOSEBEHAVIOR_H
#define EXAMPLES_INPUTVTC_TRANSFERVTCPOSEBEHAVIOR_H


/// Behavior to copy proxy pose from Vtc representation to a representation.
class TransferVtcPoseBehavior : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	/// \param	from	VtcRigidRepresentation to get the pose
	/// \param	to		Representation to set the pose
	TransferVtcPoseBehavior(const std::string& name,
								 std::shared_ptr<SurgSim::Physics::VtcRigidRepresentation> from,
								 std::shared_ptr<SurgSim::Framework::Representation> to) :
		SurgSim::Framework::Behavior(name),
		m_from(from),
		m_to(to)
	{
	}

	/// Update the behavior
	/// \param dt	The length of time (seconds) between update calls.
	virtual void update(double dt)
	{
		m_to->setPose(m_from->getCurrentVtcState().getPose());
	}

protected:
	/// Initialize the behavior
	virtual bool doInitialize()
	{
		return true;
	}
	/// Wakeup the behavior, which copies the initial pose
	virtual bool doWakeUp()
	{
		m_to->setInitialPose(m_from->getInitialPose());
		return true;
	}

private:
	/// Representation to get the pose
	std::shared_ptr<SurgSim::Physics::VtcRigidRepresentation> m_from;
	/// Representation to set the pose
	std::shared_ptr<SurgSim::Framework::Representation> m_to;
};

#endif //EXAMPLES_INPUTVTC_TRANSFERVTCPOSEBEHAVIOR_H
