#ifndef SURGSIM_GRAPHICS_VIEW_COMPONENT_H
#define SURGSIM_GRAPHICS_VIEW_COMPONENT_H

#include <SurgSim/Framework/Component.h>

#include <memory>

namespace SurgSim 
{
	namespace Graphics
	{
		class CameraActor;
		class ViewImplementation;

		class ViewComponent : public Framework::Component
		{
		public:
			ViewComponent(const std::string& name, std::shared_ptr<ViewImplementation> implementation);
			virtual ~ViewComponent();

			bool setCamera(std::shared_ptr<CameraActor> camera);

			std::shared_ptr<CameraActor> getCamera() const
			{
				return m_camera;
			}

			void setScreen(unsigned int screen);
			void setFullScreen(bool fullscreen);
			void setWindow(unsigned int x, unsigned int y, unsigned int width, unsigned int height);

			/// \param	dt	The time in seconds of the preceding timestep.
			void update(double dt)
			{
				if (m_isFirstUpdate)
				{
					doBeforeFirstUpdate(dt);
				}
				
				doUpdate(dt);
				m_isFirstUpdate = false;
			}

			std::shared_ptr<ViewImplementation> getImplementation()
			{
				return m_implementation;
			}

		private:
			virtual bool doInitialize();
			virtual bool doWakeUp();

			virtual void doBeforeFirstUpdate(double dt);
			virtual void doUpdate(double dt);

			virtual void updateWindow();

			std::shared_ptr<ViewImplementation> m_implementation;
			std::shared_ptr<CameraActor> m_camera;
			unsigned int m_screen;
			bool m_isFullscreen;
			unsigned int m_windowX;
			unsigned int m_windowY;
			unsigned int m_windowWidth;
			unsigned int m_windowHeight;

			bool m_isFirstUpdate;
		};
	}
}

#endif