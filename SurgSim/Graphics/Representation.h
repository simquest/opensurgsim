#ifndef SURGSIM_GRAPHICS_REPRESENTATION_H
#define SURGSIM_GRAPHICS_REPRESENTATION_H

#include <SurgSim/Framework/Representation.h>

namespace SurgSim 
{
	namespace Graphics
	{
		class Actor;

		class Representation : public Framework::Representation
		{
		public:
			Representation(std::shared_ptr<Actor> actor);
			virtual ~Representation();

			std::shared_ptr<Actor> getActor() const
			{
				return m_actor;
			}

		private:
			std::shared_ptr<Actor> m_actor;
		};
	}
}

#endif