#ifndef SURGSIM_GRAPHICS_OSG_CAMERA_IMPLEMENTATION_H
#define SURGSIM_GRAPHICS_OSG_CAMERA_IMPLEMENTATION_H

#include <SurgSim/Graphics/CameraImplementation.h>

#include <osg/Camera>

namespace SurgSim 
{
	namespace Graphics
	{
		namespace Osg
		{
			class CameraImplementation : public SurgSim::Graphics::CameraImplementation
			{
			public:
				CameraImplementation();
				virtual ~CameraImplementation();

				osg::ref_ptr<osg::Camera> getCamera() const
				{
					return m_camera;
				}
			private:
				virtual void doUpdate(double dt)
				{
				}

				osg::ref_ptr<osg::Camera> m_camera;
			};
		}
	}
}

#endif