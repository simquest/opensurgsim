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

#ifndef SURGSIM_BLOCKS_TRANSFERTRACKIRPOSEBEHAVIOR_H
#define SURGSIM_BLOCKS_TRANSFERTRACKIRPOSEBEHAVIOR_H

#include <string>
#include "SurgSim/Framework/Behavior.h"

namespace SurgSim
{

namespace Input
{
	class InputComponent;
}

namespace Framework
{
	class Representation;
}

namespace Graphics
{
	class OsgCamera;
}

namespace Blocks
{

class TransferTrackIRPoseBehavior : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	/// \param	from	Representation to get the pose
	/// \param	to		Representation to set the pose
	/// \param	poseName Name of the pose data in the input to transfer
	TransferTrackIRPoseBehavior(const std::string& name, std::shared_ptr<SurgSim::Input::InputComponent> from,
							  std::shared_ptr<SurgSim::Graphics::OsgCamera> to,
							  const std::string& poseName = "pose");

	/// Update the behavior
	/// \param dt	The length of time (seconds) between update calls.
	virtual void update(double dt);

protected:
	/// Initialize the behavior
	virtual bool doInitialize();

	/// Wakeup the behavior
	virtual bool doWakeUp();

private:
	/// Representation to get the pose
	std::shared_ptr<SurgSim::Input::InputComponent> m_from;
	/// Representation to set the pose
	std::shared_ptr<SurgSim::Graphics::OsgCamera> m_to;

	std::string m_poseName;
};


};  // namespace Blocks

};  // namespace SurgSim

#endif  //SURGSIM_BLOCKS_TRANSFERTRACKIRPOSEBEHAVIOR_H
