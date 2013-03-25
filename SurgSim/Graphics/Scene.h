#ifndef SURGSIM_GRAPHICS_SCENE_H
#define SURGSIM_GRAPHICS_SCENE_H

#include <memory>
#include <string>
#include <unordered_map>

namespace SurgSim 
{
	namespace Graphics
	{
		class Actor;
		class Representation;
		class SceneImplementation;

		class Scene
		{
		public:
			Scene(std::shared_ptr<SceneImplementation> implementation);
			virtual ~Scene();

			bool addActor(std::shared_ptr<Actor> actor);
			bool removeActor(std::shared_ptr<Actor> actor);
			bool removeActor(const std::string& name);

			void update(double dt)
			{
				doUpdate(dt);
			}

			std::shared_ptr<SceneImplementation> getImplementation() const
			{
				return m_implementation;
			}

		private:
			std::unordered_map<std::string, std::shared_ptr<Actor>> m_actors;
			std::shared_ptr<SceneImplementation> m_implementation;

			virtual void doUpdate(double dt);
		};
	}
}

#endif