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

#include <string>
#include <sstream>
#include <stdlib.h>

#include <Examples/BouncingBalls/AddRandomSphereBehavior.h>

#include <SurgSim/Framework/Behavior.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Blocks/SphereElement.h>
#include <SurgSim/Math/Vector.h>

using SurgSim::Blocks::SphereElement;
using SurgSim::Framework::Behavior;
using SurgSim::Framework::SceneElement;
using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Blocks
{

AddRandomSphereBehavior::AddRandomSphereBehavior():
	Behavior("DynamicallyAddSphereElement"), m_totalTime(0.0), m_numElements(0)
{
}


AddRandomSphereBehavior::~AddRandomSphereBehavior()
{
}

bool AddRandomSphereBehavior::doInitialize()
{
	return true;
}

bool AddRandomSphereBehavior::doWakeUp()
{
	return true;
}

void AddRandomSphereBehavior::update(double dt)
{
	m_totalTime += dt;

	if (m_totalTime > 3.0)
	{
		m_totalTime = 0.0;

		std::stringstream ss;
		ss << ++ m_numElements;

		std::srand(static_cast<unsigned int>(std::time(0)));
		double m_x = static_cast<double>(std::rand() % 10) / 10.0;
		double m_y = static_cast<double>(std::rand() % 2) + 1.0;
		double m_z = static_cast<double>(std::rand() % 10) / 10.0;

		std::string name = "sphereId_" + ss.str();
		SurgSim::Math::RigidTransform3d pose = SurgSim::Math::makeRigidTransform
			(SurgSim::Math::Quaterniond::Identity(), Vector3d(m_x, m_y, m_z));

		std::shared_ptr<SceneElement> m_element = std::make_shared<SphereElement>(name, pose);
		getScene()->addSceneElement(m_element);
	}
}

};  // namespace Blocks

};  // namespace SurgSim