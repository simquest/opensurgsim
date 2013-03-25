#ifndef SURGSIM_GRAPHICS_VIEW_ELEMENT_H
#define SURGSIM_GRAPHICS_VIEW_ELEMENT_H

#include <SurgSim/Framework/SceneElement.h>

namespace SurgSim 
{
	namespace Graphics
	{
		class CameraElement;
		class ViewComponent;

		class ViewElement : public Framework::SceneElement
		{
		public:
			ViewElement(const std::string& name, std::shared_ptr<SurgSim::Graphics::ViewComponent> component);
			virtual ~ViewElement();

			bool setCamera(std::shared_ptr<CameraElement> camera);

			std::shared_ptr<CameraElement> getCamera() const
			{
				return m_camera;
			}

			void setScreen(unsigned int screen);
			void setFullScreen(bool fullscreen);
			void setWindow(unsigned int x, unsigned int y, unsigned int width, unsigned int height);
			
		private:
			virtual bool doInitialize();
			virtual bool doWakeUp();

			std::shared_ptr<CameraElement> m_camera;
			std::shared_ptr<ViewComponent> m_view;
		};
	}
}

#endif