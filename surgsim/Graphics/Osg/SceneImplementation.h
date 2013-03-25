#ifndef SURGSIM_GRAPHICS_OSG_SCENE_IMPLEMENTATION_H
#define SURGSIM_GRAPHICS_OSG_SCENE_IMPLEMENTATION_H

#include <SurgSim/Graphics/SceneImplementation.h>

#include <memory>
#include <string>
#include <set>

#include <osg/Group>

namespace SurgSim 
{
	namespace Graphics
	{
		class Actor;
		class Representation;
		class ViewImplementation;

		namespace Osg
		{
			class ViewImplementation;

			class SceneImplementation : public SurgSim::Graphics::SceneImplementation
			{
			public:
				SceneImplementation();
				virtual ~SceneImplementation();

			private:
				virtual bool doAddActor(std::shared_ptr<SurgSim::Graphics::ActorImplementation> actor);
				virtual bool doRemoveActor(std::shared_ptr<SurgSim::Graphics::ActorImplementation> actor);

				virtual void doUpdate(double dt);

				osg::ref_ptr<osg::Group> m_root;

				std::set<std::shared_ptr<SurgSim::Graphics::ViewImplementation>> m_views;
			};
		}
	}
}

#endif