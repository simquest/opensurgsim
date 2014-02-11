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

#ifndef SURGSIM_BLOCKS_STAPLEELEMENT_H
#define SURGSIM_BLOCKS_STAPLEELEMENT_H

#include "SurgSim/Math/RigidTransform.h"

#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"


namespace SurgSim
{

namespace Blocks
{

class StapleElement : public SurgSim::Framework::SceneElement
{
public:
	/// Constructor
	/// \param name Name of the staple element.
	explicit StapleElement(const std::string& name);
	
	/// Set initial pose of the staple
	/// \param pose	The initial pose to set. 
	void setPose(const SurgSim::Math::RigidTransform3d& pose);

	/// Destructor
	~StapleElement();


protected:
	/// Initialize the behavior
	virtual bool doInitialize();

	/// Wakeup the behavior
	virtual bool doWakeUp();


private:
	std::string m_name;
	SurgSim::Math::RigidTransform3d m_pose;

};


};
};

#endif //SURGSIM_BLOCKS_STAPLEELEMENT_H
