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
	}
}

#endif
