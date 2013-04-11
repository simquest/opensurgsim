#ifndef SURGSIM_PHYSICS_ACTOR_H
#define SURGSIM_PHYSICS_ACTOR_H

#include <SurgSim/Framework/Representation.h>

#include <memory>
#include <string>

namespace SurgSim 
{
	namespace Physics
	{
		class ActorImplementation;

		class Actor : public SurgSim::Framework::Representation
		{
		public:
			Actor(const std::string& name, std::shared_ptr<ActorImplementation> implementation);
			virtual ~Actor();

			/// Query if this object is active in the scene.
			/// \return	true if active, false if not.
			bool isActive() 
			{
				return doIsActive();
			}

			/// Called before the scene does its physics update, lets the actor
			/// do some preprocessing
			/// \param	dt	The time in seconds that the update call will advance the scene.
			void beforeUpdate(double dt)
			{
				doBeforeUpdate(dt);
			}

			/// Called after the scene update has concluded.
			/// \param	dt	The time in seconds of the preceding timestep.
			void afterUpdate(double dt) 
			{
				doAfterUpdate(dt);
			}

			std::shared_ptr<ActorImplementation> getImplementation()
			{
				return m_implementation;
			}

		private:
			virtual bool doIsActive() = 0;

			// By Default do nothing
			virtual void doBeforeUpdate(double dt) {};
			virtual void doAfterUpdate(double dt) {};

			std::string m_name;
			std::shared_ptr<ActorImplementation> m_implementation;
		};
	}
}

#endif