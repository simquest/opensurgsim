#ifndef SURGSIM_GRAPHICS_OSG_VIEW_IMPLEMENTATION_H
#define SURGSIM_GRAPHICS_OSG_VIEW_IMPLEMENTATION_H

#include <SurgSim/Graphics/ViewImplementation.h>

#include <osgViewer/Viewer>

namespace SurgSim 
{
	namespace Graphics
	{
		class CameraImplementation;

		namespace Osg
		{
			class ViewImplementation : public SurgSim::Graphics::ViewImplementation
			{
			public:
				ViewImplementation();
				virtual ~ViewImplementation();

				virtual bool setCamera(std::shared_ptr<SurgSim::Graphics::CameraImplementation> camera);

				virtual bool setUpFullScreen(unsigned int screen = 0);
				virtual bool setUpWindow(unsigned int x, unsigned int y, unsigned int width, unsigned int height,
					unsigned int screen = 0);

				virtual bool makeFullScreened(unsigned int screen = 0);
				virtual bool makeWindowed(unsigned int x, unsigned int y, unsigned int width, unsigned int height, 
					unsigned int  = 0);

			private:
				osg::ref_ptr<osgViewer::Viewer> m_viewer;

				virtual void doUpdate(double dt);
			};
		}
	}
}

#endif