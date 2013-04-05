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

#ifndef SURGSIM_PHYSICS_MANAGER_H
#define SURGSIM_PHYSICS_MANAGER_H

#include <SurgSim/Framework/BasicThread.h>

#include <memory>
#include <set>

namespace SurgSim 
{

namespace Framework
{
	class Component;
}

namespace Physics
{
	class Scene;

	class Manager : public SurgSim::Framework::BasicThread
	{
	public:
		Manager(std::shared_ptr<Scene> scene);
		virtual ~Manager();

		virtual bool addComponent(std::shared_ptr<SurgSim::Framework::Component> component);
		virtual bool removeComponent(std::shared_ptr<SurgSim::Framework::Component> component);

	private:

		virtual bool doInitialize();
		virtual bool doStartUp();
		virtual bool doUpdate(double dt);

		std::shared_ptr<Scene> m_scene;
	};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_MANAGER_H
