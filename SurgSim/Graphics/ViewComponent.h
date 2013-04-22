// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_VIEW_COMPONENT_H
