#ifndef SURGSIM_GRAPHICS_VIEW_IMPLEMENTATION_H
#define SURGSIM_GRAPHICS_VIEW_IMPLEMENTATION_H

#include <memory>

namespace SurgSim 
{
	namespace Graphics
	{
		class CameraImplementation;

		class ViewImplementation
		{
		public:
			ViewImplementation()
			{
			}
			virtual ~ViewImplementation()
			{
			}

			bool setCamera(std::shared_ptr<CameraImplementation> cameraImplementation)
			{
				m_cameraImplementation = cameraImplementation;
				return true;
			}
			std::shared_ptr<CameraImplementation> getCamera()
			{
				return m_cameraImplementation;
			}

			virtual bool setUpFullScreen(unsigned int screen = 0) = 0;
			virtual bool setUpWindow(unsigned int x, unsigned int y, unsigned int width, unsigned int height,
				unsigned int screen = 0) = 0;

			virtual bool makeFullScreened(unsigned int screen = 0) = 0;
			virtual bool makeWindowed(unsigned int x, unsigned int y, unsigned int width, unsigned int height, 
				unsigned int screen = 0) = 0;

			void update(double dt)
			{
				doUpdate(dt);
			}
		private:
			virtual void doUpdate(double dt) = 0;

			std::shared_ptr<CameraImplementation> m_cameraImplementation;
		};
	}
}

#endif