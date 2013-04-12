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
	ViewImplementation();
	virtual ~ViewImplementation();

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

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_VIEW_IMPLEMENTATION_H
