#ifndef SURGSIM_GRAPHICS_ACTOR_H
#define SURGSIM_GRAPHICS_ACTOR_H

#include <memory>
#include <string>

namespace SurgSim 
{
	namespace Graphics
	{
		class ActorImplementation;

		class Actor
		{
		public:
			Actor(const std::string& name, std::shared_ptr<ActorImplementation> implementation);
			virtual ~Actor();

			const std::string& getName() const
			{
				return m_name;
			}

			/// \param	dt	The time in seconds of the preceding timestep.
			void update(double dt)
			{ 
				doUpdate(dt);
			}

			std::shared_ptr<ActorImplementation> getImplementation() const
			{
				return m_implementation;
			}

		private:
			std::string m_name;
			std::shared_ptr<ActorImplementation> m_implementation;

			virtual void doUpdate(double dt);
		};
	}
}

#endif