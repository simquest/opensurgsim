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

#ifndef SURGSIM_BLOCKS_TRANSFERPOSEBEHAVIOR_H
#define SURGSIM_BLOCKS_TRANSFERPOSEBEHAVIOR_H

#include <SurgSim/Framework/Behavior.h>
#include <SurgSim/Graphics/Representation.h>

using SurgSim::Math::RigidTransform3d;

namespace SurgSim
{

namespace Blocks
{

/// Behavior to copy a pose from one representation to another.
/// For example, this behavior is used to send pose updates from physics to graphics.
class TransferPoseBehavior : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	/// \param	from	Representation to get the pose
	/// \param	to	Representation to set the pose
	TransferPoseBehavior(const std::string& name, std::shared_ptr<SurgSim::Framework::Representation> from,
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
		m_to->setPose(m_from->getPose());
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
	std::shared_ptr<SurgSim::Framework::Representation> m_from;
	/// Representation to set the pose
	std::shared_ptr<SurgSim::Framework::Representation> m_to;
};


};  // namespace Blocks

};  // namespace SurgSim

#endif  //SURGSIM_BLOCKS_TRANSFERPOSEBEHAVIOR_H
