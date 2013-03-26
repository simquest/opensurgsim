#ifndef SURGSIM_PHYSICS_SCENE_IMPLEMENTATION_H
#define SURGSIM_PHYSICS_SCENE_IMPLEMENTATION_H

#include <memory>

namespace SurgSim 
{
	namespace Physics
	{
		class ActorImplementation;
		class Representation;
		class ViewImplementation;

		class SceneImplementation
		{
		public:
			SceneImplementation();
			virtual ~SceneImplementation();

			virtual bool addActor(std::shared_ptr<ActorImplementation> actor) = 0;
			virtual bool removeActor(std::shared_ptr<ActorImplementation> actor) = 0;

			virtual void update(double dt) = 0;

		private:
		};
	}
}

#endif