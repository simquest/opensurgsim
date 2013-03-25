#ifndef SURGSIM_GRAPHICS_CAMERA_IMPLEMENTATION_H
#define SURGSIM_GRAPHICS_CAMERA_IMPLEMENTATION_H

#include <SurgSim/Graphics/ActorImplementation.h>

namespace SurgSim 
{
	namespace Graphics
	{
		class CameraImplementation : public ActorImplementation
		{
		public:
			CameraImplementation();
			virtual ~CameraImplementation();
		};
	}
}

#endif