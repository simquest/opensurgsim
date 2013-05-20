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

#ifndef SURGSIM_PHYSICS_PHYSICSMANAGER_H
#define SURGSIM_PHYSICS_PHYSICSMANAGER_H

#include <memory>
#include <vector>

#include <SurgSim/Framework/ComponentManager.h>


namespace SurgSim 
{
namespace Framework
{
	class Logger;
	class Component;
}

namespace Physics
{

class RigidActorBase;
class FreeMotionStep;

class PhysicsManager : public SurgSim::Framework::ComponentManager
{
public:
	PhysicsManager();
	~PhysicsManager();

	bool addComponent(std::shared_ptr<SurgSim::Framework::Component> component);
	bool removeComponent(std::shared_ptr<SurgSim::Framework::Component> component);

protected:

	virtual bool doInitialize();
	virtual bool doStartUp();
	virtual bool doUpdate(double dt);

	template<class T>
	std::shared_ptr<T> doAddComponent(std::shared_ptr<SurgSim::Framework::Component> component, std::vector<std::shared_ptr<T>>* container);

	template<class T>
	bool doRemoveComponent(std::shared_ptr<SurgSim::Framework::Component> component, std::vector<std::shared_ptr<T>>* container);

private:

	std::shared_ptr< std::vector<std::shared_ptr<RigidActorBase>> > m_rigidActors;
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;

	std::unique_ptr<FreeMotionStep> m_freeMotionStep;

};

#include <SurgSim/Physics/PhysicsManager-inl.h>

}; // namespace Physics
}; // namespace SurgSim



#endif