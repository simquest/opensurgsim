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

#ifndef SURGSIM_GRAPHICS_VIEW_ELEMENT_H
#define SURGSIM_GRAPHICS_VIEW_ELEMENT_H

#include <SurgSim/Framework/SceneElement.h>

namespace SurgSim 
{

namespace Graphics
{

class CameraRepresentation;
class ViewComponent;

class ViewElement : public Framework::SceneElement
{
public:
	ViewElement(const std::string& name, std::shared_ptr<SurgSim::Graphics::ViewComponent> component);
	virtual ~ViewElement();

	bool setCamera(std::shared_ptr<CameraRepresentation> camera);

	std::shared_ptr<CameraRepresentation> getCamera() const
	{
		return m_camera;
	}

	void setScreen(unsigned int screen);
	void setFullScreen(bool fullscreen);
	void setWindow(unsigned int x, unsigned int y, unsigned int width, unsigned int height);
			
private:
	virtual bool doInitialize();
	virtual bool doWakeUp();

	std::shared_ptr<CameraRepresentation> m_camera;
	std::shared_ptr<ViewComponent> m_view;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_VIEW_ELEMENT_H
