#ifndef SURGSIM_PHYSICS_SCENE_H
#define SURGSIM_PHYSICS_SCENE_H

#include <memory>
#include <string>
#include <unordered_map>

namespace SurgSim 
{
	namespace Physics
	{
		class Actor;
		class Representation;
		class SceneImplementation;

		class Scene
		{
		public:
			Scene(std::shared_ptr<SceneImplementation> implementation);
			virtual ~Scene();

			virtual bool addActor(std::shared_ptr<Actor> actor);
			virtual bool removeActor(std::shared_ptr<Actor> actor);
			virtual bool removeActor(const std::string& name);

			void update(double dt)
			{
				doBeforeUpdate(dt);
				doUpdate(dt);
				doAfterUpdate(dt);
			}

		private:
			virtual void doBeforeUpdate(double dt);
			virtual void doUpdate(double dt);
			virtual void doAfterUpdate(double dt);

			std::unordered_map<std::string, std::shared_ptr<Actor>> m_actors;
			std::shared_ptr<SceneImplementation> m_implementation;
		};
	}
}

#endif