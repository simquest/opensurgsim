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

#ifndef SURGSIM_BLOCKS_BASICSCENEELEMENT_H
#define SURGSIM_BLOCKS_BASICSCENEELEMENT_H

#include "SurgSim/Framework/SceneElement.h"

namespace SurgSim
{

namespace Blocks
{

/// Simple concrete implementation of a scene element that does not have any higher logic
class BasicSceneElement : public SurgSim::Framework::SceneElement
{
public:
	/// Constructor
	/// \name	Name of the scene element
	explicit BasicSceneElement(const std::string& name);
	/// Destructor
	virtual ~BasicSceneElement();

protected:
	/// Initializes the scene element
	/// \return	True if succeeds, false if fails
	virtual bool doInitialize() override;
	/// Wakes up the scene element
	/// \pre	All scene elements are initialized
	/// \return	True if succeeds, false if fails
	virtual bool doWakeUp() override;
};

};  // namespace Blocks
};  // namespace SurgSim

#endif  // SURGSIM_BLOCKS_BASICSCENEELEMENT_H
