#ifndef SURGSIM_GRAPHICS_CAMERA_ELEMENT_H
#define SURGSIM_GRAPHICS_CAMERA_ELEMENT_H

#include <SurgSim/Graphics/Representation.h>

#include <memory>

namespace SurgSim 
{
	namespace Graphics
	{
		class CameraActor;

		class CameraElement : public Representation
		{
		public:
			CameraElement(std::shared_ptr<CameraActor> actor);
			virtual ~CameraElement();

			std::shared_ptr<CameraActor> getCameraActor() const;

		private:
		};
	}
}

#endif