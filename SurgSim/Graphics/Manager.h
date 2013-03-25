#ifndef SURGSIM_GRAPHICS_MANAGER_H
#define SURGSIM_GRAPHICS_MANAGER_H

#include <SurgSim/Framework/BasicThread.h>

#include <memory>
#include <set>

namespace SurgSim 
{
	namespace Framework
	{
		class Component;
	}

	namespace Graphics
	{
		class Scene;
		class ViewComponent;

		class Manager : public SurgSim::Framework::BasicThread
		{
		public:
			Manager(std::shared_ptr<Scene> scene);
			virtual ~Manager();

			bool addComponent(std::shared_ptr<SurgSim::Framework::Component> component);
			bool removeComponent(std::shared_ptr<SurgSim::Framework::Component> component);

		private:

			virtual bool doInitialize();
			virtual bool doStartUp();
			virtual bool doUpdate(double dt);

			std::shared_ptr<Scene> m_scene;
			std::set< std::shared_ptr<ViewComponent> > m_views;
		};
	}
}

#endif
