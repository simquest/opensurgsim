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

#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Component.h>
#include <SurgSim/Physics/PhysicsManager.h>
#include <SurgSim/Physics/FreeMotion.h>`
#include <SurgSim/Physics/Actor.h>
#include <SurgSim/Physics/DcdCollision.h>#include <SurgSim/Framework/Log.h>

namespace SurgSim
{
namespace Physics
{

PhysicsManager::PhysicsManager()
{

}

PhysicsManager::~PhysicsManager()
{

}

bool PhysicsManager::doInitialize()
{
	m_logger = getRuntime()->getLogger("PhysicsManager");
	m_actors = std::make_shared<std::vector<std::shared_ptr<Actor>>>();
	m_freeMotionStep.reset(new FreeMotion(m_actors));
	return m_logger != nullptr && m_actors != nullptr && m_freeMotionStep != nullptr;
}


bool PhysicsManager::doStartUp()
{
	return true;
}


bool PhysicsManager::addComponent(std::shared_ptr<SurgSim::Framework::Component> component)
{
	return tryAddComponent(component,m_actors.get()) != nullptr;
}

bool PhysicsManager::removeComponent(std::shared_ptr<SurgSim::Framework::Component> component)
{
	return tryRemoveComponent(component, m_actors.get());
}

bool PhysicsManager::doUpdate(double dt)
{
	m_freeMotionStep->update(dt);
	return true;
}


}; // Physics
}; // SurgSim
