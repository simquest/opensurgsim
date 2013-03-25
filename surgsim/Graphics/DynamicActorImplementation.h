#ifndef SURGSIM_GRAPHICS_ACTOR_IMPLEMENTATION_H
#define SURGSIM_GRAPHICS_ACTOR_IMPLEMENTATION_H

namespace SurgSim 
{
	namespace Graphics
	{
		class ActorImplementation
		{
		public:
			ActorImplementation();
			virtual ~ActorImplementation();

			void update(double dt)
			{
				doUpdate(dt);
			}

		private:
			virtual void doUpdate(double dt) = 0;
		};
	}
}

#endif