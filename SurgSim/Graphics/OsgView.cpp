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

#include "SurgSim/Graphics/OsgView.h"


#include "SurgSim/Devices/Keyboard/KeyboardDevice.h"
#include "SurgSim/Devices/Keyboard/OsgKeyboardHandler.h"
#include "SurgSim/Devices/Mouse/MouseDevice.h"
#include "SurgSim/Devices/Mouse/OsgMouseHandler.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgConversions.h"
#include "SurgSim/Graphics/OsgTrackballZoomManipulator.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Framework/PoseComponent.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/RigidTransform.h"

#include <osgViewer/ViewerEventHandlers>
#include <osg/DisplaySettings>


using SurgSim::Graphics::OsgCamera;
using SurgSim::Graphics::OsgView;
using SurgSim::Framework::checkAndConvert;

namespace
{

// Mapping from OSS values to OSG values, the order here needs to match the numerical value in
// SurgSim::Graphics::View::StereoMode
const osg::DisplaySettings::StereoMode StereoModeEnums[SurgSim::Graphics::View::STEREO_MODE_COUNT] =
{
	osg::DisplaySettings::QUAD_BUFFER,
	osg::DisplaySettings::ANAGLYPHIC,
	osg::DisplaySettings::HORIZONTAL_SPLIT,
	osg::DisplaySettings::VERTICAL_SPLIT,
	osg::DisplaySettings::LEFT_EYE,
	osg::DisplaySettings::RIGHT_EYE,
	osg::DisplaySettings::HORIZONTAL_INTERLACE,
	osg::DisplaySettings::VERTICAL_INTERLACE,
	osg::DisplaySettings::CHECKERBOARD
};

// Mapping from OSS values to OSG values, the order here needs to match the numerical value in
// SurgSim::Graphics::View::DisplayType
const osg::DisplaySettings::DisplayType DisplayTypeEnums[SurgSim::Graphics::View::DISPLAY_TYPE_COUNT] =
{
	osg::DisplaySettings::MONITOR,
	osg::DisplaySettings::HEAD_MOUNTED_DISPLAY
};

}

namespace SurgSim
{
namespace Graphics
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Graphics::OsgView, OsgView);

OsgView::OsgView(const std::string& name) : View(name),
	m_isWindowBorderEnabled(true),
	m_isFirstUpdate(true),
	m_areWindowSettingsDirty(false),
	m_view(new osgViewer::View()),
	m_osgMapUniforms(false),
	m_manipulatorPosition(SurgSim::Math::Vector3d(0.0, 0.0, -3.0)),
	m_manipulatorLookat(SurgSim::Math::Vector3d::Zero()),
	m_keyboardEnabled(false),
	m_mouseEnabled(false)
{
	m_position[0] = 0;
	m_position[1] = 0;
	m_dimensions[0] = 1024;
	m_dimensions[1] = 768;

	/// Clear the OSG default camera, let that be handled at a higher level.
	m_view->setCamera(nullptr);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(OsgView, bool, CameraManipulatorEnabled,
									  isManipulatorEnabled, enableManipulator);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(OsgView, SurgSim::Math::Vector3d, CameraPosition,
									  getManipulatorPosition, setManipulatorPosition);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(OsgView, SurgSim::Math::Vector3d, CameraLookAt,
									  getManipulatorLookAt, setManipulatorLookAt);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(OsgView, bool, OsgMapUniforms, getOsgMapsUniforms, setOsgMapsUniforms);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(OsgView, bool, KeyboardDeviceEnabled,
									  isKeyboardDeviceEnabled, enableKeyboardDevice);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(OsgView, bool, MouseDeviceEnabled,
									  isMouseDeviceEnabled, enableMouseDevice);
}

OsgView::~OsgView()
{
	// Clean up the handler callbacks
	enableKeyboardDevice(false);
	enableMouseDevice(false);
}

void OsgView::setPosition(const std::array<int, 2>& position)
{
	if (position != m_position)
	{
		m_position = position;
		m_areWindowSettingsDirty = true;
	}
}

std::array<int, 2> OsgView::getPosition() const
{
	return m_position;
}

void OsgView::setDimensions(const std::array<int, 2>& dimensions)
{
	if (m_dimensions != dimensions)
	{
		m_areWindowSettingsDirty = true;
		m_dimensions = dimensions;
	}
}

std::array<int, 2> OsgView::getDimensions() const
{
	return m_dimensions;
}

void OsgView::setWindowBorderEnabled(bool enabled)
{
	m_isWindowBorderEnabled = enabled;
	m_areWindowSettingsDirty = true;
}

bool OsgView::isWindowBorderEnabled() const
{
	return m_isWindowBorderEnabled;
}

void OsgView::setCamera(std::shared_ptr<SurgSim::Framework::Component> camera)
{
	auto osgCamera = checkAndConvert<OsgCamera>(camera, "SurgSim::Graphics::OsgCamera");

	View::setCamera(camera);
	m_view->setCamera(osgCamera->getOsgCamera());
	osgCamera->setViewport(0, 0, m_dimensions[0], m_dimensions[1]);
}

void OsgView::update(double dt)
{
	if (isActive() && m_areWindowSettingsDirty)
	{
		osg::Camera* viewCamera = m_view->getCamera();
		if (viewCamera)
		{
			osgViewer::GraphicsWindow* window =
				dynamic_cast<osgViewer::GraphicsWindow*>(viewCamera->getGraphicsContext());
			if (window)
			{
				window->setWindowDecoration(m_isWindowBorderEnabled);
				window->setWindowRectangle(m_position[0], m_position[1], m_dimensions[0], m_dimensions[1]);
				m_areWindowSettingsDirty = false;
			}
		}
	}
	if (isManipulatorEnabled())
	{
		auto matrix = m_view->getCameraManipulator()->getMatrix();
		auto pose = Math::RigidTransform3d(fromOsg(matrix));
		getSceneElement()->getPoseComponent()->setPose(pose);
	}
}

bool OsgView::doInitialize()
{
	return true;
}

bool OsgView::doWakeUp()
{
	osg::ref_ptr<osg::DisplaySettings> displaySettings = new osg::DisplaySettings;
	displaySettings->setDefaults();

	if (isStereo())
	{
		displaySettings->setStereo(isStereo());
		displaySettings->setStereoMode(StereoModeEnums[getStereoMode()]);
		displaySettings->setDisplayType(DisplayTypeEnums[getDisplayType()]);
		displaySettings->setEyeSeparation(static_cast<float>(getEyeSeparation()));
		displaySettings->setScreenDistance(static_cast<float>(getScreenDistance()));
		displaySettings->setScreenWidth(static_cast<float>(getScreenWidth()));
		displaySettings->setScreenHeight(static_cast<float>(getScreenHeight()));
	}


	m_view->setDisplaySettings(displaySettings);

	osg::ref_ptr<osgViewer::ViewConfig> viewConfig;

//  #refactor
//  HS-2014-may-14 Linux is stuck at OSG 3.2.0-rc1, this is not implemented there, they are waiting for
//  3.2.1 to move forward, implement this once linux has caught up
// 	if (isFullScreen())
// 	{
// 		viewConfig = new osgViewer::SingleScreen(getTargetScreen());
// 	}
// 	else
// 	{
// 		viewConfig = new osgViewer::SingleWindow(m_x, m_y, m_width, m_height, getTargetScreen());
// 	}
//	m_view->apply(viewConfig);

	if (isFullScreen())
	{
		m_view->setUpViewOnSingleScreen(getTargetScreen());
	}
	else
	{
		m_view->setUpViewInWindow(m_position[0], m_position[1], m_dimensions[0], m_dimensions[1], getTargetScreen());
	}


	auto statsHandler = new osgViewer::StatsHandler;
	m_view->addEventHandler(statsHandler);

	if (m_osgMapUniforms)
	{
		fixupStatsHandler(statsHandler);

		m_view->getCamera()->getGraphicsContext()->getState()->setUseModelViewAndProjectionUniforms(true);
		m_view->getCamera()->getGraphicsContext()->getState()->setUseVertexAttributeAliasing(true);
	}


	return true;
}

void OsgView::fixupStatsHandler(osgViewer::StatsHandler* statsHandler)
{
	// use ref_ptr in case loading fails we don't have to clean up
	osg::ref_ptr<osg::Shader> vertexShader = new osg::Shader(osg::Shader::VERTEX);
	osg::ref_ptr<osg::Shader> fragmentShader = new osg::Shader(osg::Shader::FRAGMENT);

	bool success = true;
	std::string fileName;
	if (getRuntime()->getApplicationData()->tryFindFile("Shaders/osg_statshandler.vert", &fileName))
	{
		success = vertexShader->loadShaderSourceFromFile(fileName);
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
				<< "Could not find Shaders/osg_statshandler.vert, the osg stats "
				<< "display will probably not work correctly.";
		success = false;
	}

	if (getRuntime()->getApplicationData()->tryFindFile("Shaders/osg_statshandler.frag", &fileName))
	{
		success = fragmentShader->loadShaderSourceFromFile(fileName);
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
				<< "Could not find Shaders/osg_statshandler.frag, the osg stats "
				<< "display will probably not work correctly.";
		success = false;
	}

	if (success)
	{
		osg::ref_ptr<osg::Program> program = new osg::Program;
		program->addShader(fragmentShader);
		program->addShader(vertexShader);

		auto state = statsHandler->getCamera()->getOrCreateStateSet();

		auto texture = new osg::Texture2D();
		texture->setTextureSize(256, 256);

		state->setAttributeAndModes(program);
		state->setTextureAttributeAndModes(0, texture);
		state->addUniform(new osg::Uniform("osg_TextTexture", static_cast<int>(0)));
	}
}

void SurgSim::Graphics::OsgView::setOsgMapsUniforms(bool val)
{
	SURGSIM_ASSERT(!isAwake()) << "Can't change mapping mode after waking up.";
	m_osgMapUniforms = val;
}

bool SurgSim::Graphics::OsgView::getOsgMapsUniforms()
{
	return m_osgMapUniforms;
}

osg::ref_ptr<osgViewer::View> SurgSim::Graphics::OsgView::getOsgView() const
{
	return m_view;
}

void SurgSim::Graphics::OsgView::enableManipulator(bool val)
{
	if (m_manipulator == nullptr)
	{
		m_manipulator = new SurgSim::Graphics::OsgTrackballZoomManipulator();
		// Set a default position
		m_manipulator->setTransformation(
			SurgSim::Graphics::toOsg(m_manipulatorPosition),
			SurgSim::Graphics::toOsg(m_manipulatorLookat),
			osg::Vec3d(0.0f, 1.0f, 0.0f));
	}

	if (val)
	{
		getOsgView()->setCameraManipulator(m_manipulator, false);
	}
	else
	{
		getOsgView()->setCameraManipulator(nullptr);
	}
}

bool SurgSim::Graphics::OsgView::isManipulatorEnabled()
{
	return getOsgView()->getCameraManipulator() != nullptr;
}

void SurgSim::Graphics::OsgView::enableKeyboardDevice(bool val)
{
	// Early return if device is already turned on/off.
	if (val == m_keyboardEnabled)
	{
		return;
	}

	std::shared_ptr<SurgSim::Input::CommonDevice> keyboardDevice = getKeyboardDevice();
	osg::ref_ptr<osgGA::GUIEventHandler> keyboardHandle =
		std::static_pointer_cast<SurgSim::Device::KeyboardDevice>(keyboardDevice)->getKeyboardHandler();
	if (val)
	{
		getOsgView()->addEventHandler(keyboardHandle);
		m_keyboardEnabled = true;
	}
	else
	{
		getOsgView()->removeEventHandler(keyboardHandle);
		m_keyboardEnabled = false;
	}
}

bool SurgSim::Graphics::OsgView::isKeyboardDeviceEnabled()
{
	return m_keyboardEnabled;
}

std::shared_ptr<SurgSim::Input::CommonDevice> SurgSim::Graphics::OsgView::getKeyboardDevice()
{
	if (m_keyboardDevice == nullptr)
	{
		m_keyboardDevice = std::make_shared<SurgSim::Device::KeyboardDevice>("Keyboard");
		if (!m_keyboardDevice->isInitialized())
		{
			m_keyboardDevice->initialize();
		}
	}
	return m_keyboardDevice;
}

void SurgSim::Graphics::OsgView::enableMouseDevice(bool val)
{
	// Early return if device is already turned on/off.
	if (val == m_mouseEnabled)
	{
		return;
	}

	std::shared_ptr<SurgSim::Input::CommonDevice> mouseDevice = getMouseDevice();
	osg::ref_ptr<osgGA::GUIEventHandler> mouseHandler =
		std::static_pointer_cast<SurgSim::Device::MouseDevice>(mouseDevice)->getMouseHandler();
	if (val)
	{
		getOsgView()->addEventHandler(mouseHandler);
		m_mouseEnabled = true;
	}
	else
	{
		getOsgView()->removeEventHandler(mouseHandler);
		m_mouseEnabled = false;
	}
}

bool SurgSim::Graphics::OsgView::isMouseDeviceEnabled()
{
	return m_mouseEnabled;
}

std::shared_ptr<SurgSim::Input::CommonDevice> SurgSim::Graphics::OsgView::getMouseDevice()
{
	if (m_mouseDevice == nullptr)
	{
		m_mouseDevice = std::make_shared<SurgSim::Device::MouseDevice>("Mouse");
		if (!m_mouseDevice->isInitialized())
		{
			m_mouseDevice->initialize();
		}
	}
	return m_mouseDevice;
}


void SurgSim::Graphics::OsgView::setManipulatorParameters(const SurgSim::Math::Vector3d& position,
		const SurgSim::Math::Vector3d& lookat)
{
	m_manipulatorPosition = position;
	m_manipulatorLookat = lookat;

	if (m_manipulator != nullptr)
	{
		m_manipulator->setTransformation(
			SurgSim::Graphics::toOsg(m_manipulatorPosition),
			SurgSim::Graphics::toOsg(m_manipulatorLookat),
			osg::Vec3d(0.0f, 1.0f, 0.0f));
	}
}

void SurgSim::Graphics::OsgView::setManipulatorPosition(const SurgSim::Math::Vector3d& position)
{
	setManipulatorParameters(position, m_manipulatorLookat);
}

SurgSim::Math::Vector3d SurgSim::Graphics::OsgView::getManipulatorPosition()
{
	return m_manipulatorPosition;
}

void SurgSim::Graphics::OsgView::setManipulatorLookAt(const SurgSim::Math::Vector3d& lookAt)
{
	setManipulatorParameters(m_manipulatorPosition, lookAt);
}

SurgSim::Math::Vector3d SurgSim::Graphics::OsgView::getManipulatorLookAt()
{
	return m_manipulatorLookat;
}

}
}