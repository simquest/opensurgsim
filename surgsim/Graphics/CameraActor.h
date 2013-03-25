#ifndef SURGSIM_GRAPHICS_CAMERA_ACTOR_H
#define SURGSIM_GRAPHICS_CAMERA_ACTOR_H

#include <SurgSim/Graphics/Actor.h>

#include <memory>

namespace SurgSim 
{
	namespace Graphics
	{
		class CameraImplementation;

		class CameraActor : public Actor
		{
		public:
			CameraActor(const std::string& name, std::shared_ptr<CameraImplementation> implementation);
			virtual ~CameraActor();

			std::shared_ptr<CameraImplementation> getCameraImplementation() const;

		private:
		};
	}
}

#endif