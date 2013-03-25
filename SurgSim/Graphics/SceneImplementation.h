#ifndef SURGSIM_GRAPHICS_SCENE_IMPLEMENTATION_H
#define SURGSIM_GRAPHICS_SCENE_IMPLEMENTATION_H

#include <memory>
#include <unordered_map>

namespace SurgSim 
{
	namespace Graphics
	{
		class ActorImplementation;
		class Representation;
		class ViewImplementation;

		class SceneImplementation
		{
		public:
			SceneImplementation();
			virtual ~SceneImplementation();

			bool addActor(std::shared_ptr<ActorImplementation> actor)
			{
				return doAddActor(actor);
			}
			bool removeActor(std::shared_ptr<ActorImplementation> actor)
			{
				return doRemoveActor(actor);
			}

			void update(double dt)
			{
				doUpdate(dt);
			}

		private:
			virtual bool doAddActor(std::shared_ptr<ActorImplementation> actor) = 0;
			virtual bool doRemoveActor(std::shared_ptr<ActorImplementation> actor) = 0;
			virtual void doUpdate(double dt) = 0;
		};
	}
}

#endif