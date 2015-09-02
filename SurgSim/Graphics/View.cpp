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

#include "SurgSim/Graphics/View.h"
#include "SurgSim/DataStructures/DataStructuresConvert.h"
#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Graphics/Camera.h"

using SurgSim::Framework::Component;
using SurgSim::Framework::checkAndConvert;

namespace SurgSim
{
namespace Graphics
{

View::View(const std::string& name) :
	SurgSim::Framework::Component(name),
	m_stereoMode(STEREO_MODE_NONE),
	m_displayType(DISPLAY_TYPE_MONITOR),
	m_targetScreen(0),
	m_isFullscreen(false),
	m_eyeSeparation(0.06),
	m_screenDistance(1.0),
	m_screenWidth(0.0),
	m_screenHeight(0.0)
{
	typedef std::array<int, 2> CoordinateType;
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(View, CoordinateType, Position, getPosition, setPosition);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(View, CoordinateType, Dimensions, getDimensions, setDimensions);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(View, std::shared_ptr<SurgSim::Framework::Component>, Camera,
									  getCamera, setCamera);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(View, bool, WindowBorder, isWindowBorderEnabled, setWindowBorderEnabled);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(View, int, StereoMode, getStereoMode, setStereoMode);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(View, int, DisplayType, getDisplayType, setDisplayType);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(View, bool, FullScreen, isFullScreen, setFullScreen);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(View, int, TargetScreen, getTargetScreen, setTargetScreen);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(View, double, EyeSeparation, getEyeSeparation, setEyeSeparation);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(View, double, ScreenDistance, getScreenDistance, setScreenDistance);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(View, double, ScreenWidth, getScreenWidth, setScreenWidth);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(View, double, ScreenHeight, getScreenHeight, setScreenHeight);
}

void View::setCamera(std::shared_ptr<Component> camera)
{
	m_camera = checkAndConvert<Camera>(camera, "SurgSim::Graphics::Camera");
	m_camera->setStereo(isStereo());
}

std::shared_ptr<Camera> View::getCamera() const
{
	return m_camera;
}

bool View::doInitialize()
{
	SURGSIM_ASSERT(m_camera != nullptr) << "View cannot be created without a camera.";
	return true;
}

bool View::isStereo() const
{
	return m_stereoMode != STEREO_MODE_NONE;
}


void View::setStereoMode(int mode)
{
	SURGSIM_ASSERT(!isAwake()) << "Can't change the view settings once the view has been woken up.";
	SURGSIM_ASSERT(mode < STEREO_MODE_COUNT) << "Invalid StereoMode " << mode;
	m_stereoMode = static_cast<StereoMode>(mode);
	if (m_camera != nullptr)
	{
		m_camera->setStereo(isStereo());
	}
}

int View::getStereoMode() const
{
	return m_stereoMode;
}

void View::setDisplayType(int type)
{
	SURGSIM_ASSERT(!isAwake()) << "Can't change the view settings once the view has been woken up.";
	SURGSIM_ASSERT(type < DISPLAY_TYPE_COUNT) << "Invalid DisplayType " << type;
	m_displayType = type;
}

int View::getDisplayType() const
{
	return m_displayType;
}

void View::setFullScreen(bool val)
{
	SURGSIM_ASSERT(!isAwake()) << "Can't change the view settings once the view has been woken up.";
	m_isFullscreen = val;
}

bool View::isFullScreen() const
{
	return m_isFullscreen;
}

void View::setTargetScreen(int val)
{
	SURGSIM_ASSERT(!isAwake()) << "Can't change the view settings once the view has been woken up.";
	m_targetScreen = val;
}

int View::getTargetScreen() const
{
	return m_targetScreen;
}

void View::setScreenDistance(double val)
{
	SURGSIM_ASSERT(!isAwake()) << "Can't change the view settings once the view has been woken up.";
	m_screenDistance = val;
}

double View::getScreenDistance() const
{
	return m_screenDistance;
}

void View::setEyeSeparation(double val)
{
	SURGSIM_ASSERT(!isAwake()) << "Can't change the view settings once the view has been woken up.";
	m_eyeSeparation = val;
}

double View::getEyeSeparation() const
{
	return m_eyeSeparation;
}

double View::getScreenWidth() const
{
	SURGSIM_ASSERT(!isAwake()) << "Can't change the view settings once the view has been woken up.";
	return m_screenWidth;
}

void View::setScreenWidth(double val)
{
	m_screenWidth = val;
}

double View::getScreenHeight() const
{
	return m_screenHeight;
}

void View::setScreenHeight(double val)
{
	SURGSIM_ASSERT(!isAwake()) << "Can't change the view settings once the view has been woken up.";
	m_screenHeight = val;
}

}
}

