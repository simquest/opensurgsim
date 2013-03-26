#ifndef SURGSIM_GRAPHICS_RIGID_ACTOR_H
#define SURGSIM_GRAPHICS_RIGID_ACTOR_H

#include "Actor.h"

#include <SurgSim/Math/RigidTransform.h>

#include <memory>
#include <string>

namespace SurgSim 
{
	namespace Graphics
	{
		class RigidActorImplementation;

		class RigidActor : public Actor
		{
		public:
			RigidActor(const std::string& name, std::shared_ptr<RigidActorImplementation> implementation);
			virtual ~RigidActor();

			void setPose(const SurgSim::Math::RigidTransform3d& pose);
			const SurgSim::Math::RigidTransform3d& getPose() const;

			std::shared_ptr<RigidActorImplementation> getRigidImplementation() const
			{
				return std::static_pointer_cast<RigidActorImplementation>(getImplementation());
			}

		};
	}
}

#endif  // SURGSIM_GRAPHICS_RIGID_ACTOR_H
