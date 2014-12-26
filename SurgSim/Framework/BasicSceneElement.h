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

#ifndef SURGSIM_FRAMEWORK_BASICSCENEELEMENT_H
#define SURGSIM_FRAMEWORK_BASICSCENEELEMENT_H

#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Framework/Macros.h"

namespace SurgSim
{

namespace Framework
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

	SURGSIM_CLASSNAME(SurgSim::Framework::BasicSceneElement);

protected:
	/// Initializes the scene element
	/// \return	True if succeeds, false if fails
	bool doInitialize() override;

};

};  // namespace Framework
};  // namespace SurgSim

#endif  // SURGSIM_FRAMEWORK_BASICSCENEELEMENT_H
