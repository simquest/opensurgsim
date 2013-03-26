#ifndef SURGSIM_GRAPHICS_RIGID_ACTOR_IMPLEMENTATION_H
#define SURGSIM_GRAPHICS_RIGID_ACTOR_IMPLEMENTATION_H

#include "ActorImplementation.h"

#include <SurgSim/Math/RigidTransform.h>

namespace SurgSim 
{
	namespace Graphics
	{
		class RigidActorImplementation : public ActorImplementation
		{
		public:
			RigidActorImplementation();
			virtual ~RigidActorImplementation();

			void setPose(const SurgSim::Math::RigidTransform3d& pose)
			{
				m_pose = pose;
			}

			const SurgSim::Math::RigidTransform3d& getPose() const
			{
				return m_pose;
			}

		private:
			virtual void doUpdate(double dt) = 0;

			SurgSim::Math::RigidTransform3d m_pose;
		};
	}
}

#endif