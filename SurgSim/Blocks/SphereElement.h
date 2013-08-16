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

#ifndef SURGSIM_BLOCKS_SPHEREELEMENT_H
#define SURGSIM_BLOCKS_SPHEREELEMENT_H

#include <SurgSim/Framework/SceneElement.h>

#include <SurgSim/Math/RigidTransform.h>


namespace SurgSim
{

namespace Blocks
{

class SphereElement : public SurgSim::Framework::SceneElement
{
public:

	explicit SphereElement(const std::string& name, const SurgSim::Math::RigidTransform3d& pose);

	~SphereElement();


protected:
	virtual bool doInitialize();

	virtual bool doWakeUp();


private:
	std::string m_name;
	SurgSim::Math::RigidTransform3d m_pose;

};


};
};

#endif