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

#ifndef SURGSIM_GRAPHICS_OSGVIEW_H
#define SURGSIM_GRAPHICS_OSGVIEW_H

#include "SurgSim/Graphics/View.h"

#include <osgViewer/Viewer>


namespace SurgSim
{

namespace Input
{
class CommonDevice;
}

namespace Graphics
{

class OsgCamera;
class OsgTrackballZoomManipulator;

/// OSG-based implementation of graphics view class.
///
/// A Graphics::OsgView wraps a osgViewer::View to provide a visualization of the scene to the user.
///
/// A Graphics::OsgCamera controls the viewpoint of this View.
class OsgView : public View
{
public:
	/// Constructor
	/// \post	The view has no camera.
	/// \post	The position of the view is (0, 0).
	/// \post	The dimensions of the view are 800 x 600.
	/// \post	The window border is enabled.
	/// \param	name	Name of the view
	explicit OsgView(const std::string& name);

	/// Destructor
	~OsgView();

	virtual bool setPosition(int x, int y) override;

	virtual void getPosition(int* x, int* y) const override;

	virtual bool setDimensions(int width, int height) override;

	virtual void getDimensions(int* width, int* height) const override;

	virtual void setWindowBorderEnabled(bool enabled) override;

	virtual bool isWindowBorderEnabled() const override;

	/// Sets the camera which provides the viewpoint in the scene
	/// Only allows OsgCamera components, any other will not be set and it will return false.
	/// \param	camera	Camera whose image will be shown in this view
	/// \return	True if it succeeded, false if it failed
	virtual bool setCamera(std::shared_ptr<Camera> camera) override;

	/// Enables a camera manipulator, implemented via a trackball, this is a temporary solution as it uses
	/// the OSG input events rather than reading from the OpenSurgSim input.
	/// \param val whether to enable the manipulator or not.
	void enableManipulator(bool val);

	/// As the camera is not accessible from here and as it cannot be controlled from the outside
	/// any more we let the user set the parameters from here.
	/// \param	position	The position of the camera.
	/// \param	lookat  	The location the camera looks at.
	void setManipulatorParameters(SurgSim::Math::Vector3d position, SurgSim::Math::Vector3d lookat);

	/// Return the keyboard to be used with this view.
	/// \return A keyboard device
	virtual std::shared_ptr<SurgSim::Input::CommonDevice> getKeyboardDevice();

	/// Turn on/off the keyboard device to be used.
	/// \param val Indicate whether or not to use keyboard device
	virtual void enableKeyboardDevice(bool val);

	/// Return the mouse to be used with this view.
	/// \return A mouse device
	virtual std::shared_ptr<SurgSim::Input::CommonDevice> getMouseDevice();

	/// Turn on/off the mouse device to be used.
	/// \param val Indicate whether or not to use mouse device
	virtual	void enableMouseDevice(bool val);

	virtual void update(double dt) override;

	/// \return the OSG view which performs the actual work involved in setting up and rendering to a window
	osg::ref_ptr<osgViewer::View> getOsgView() const;



protected:
	/// Initialize the view
	/// \post The view's window is setup.
	virtual bool doInitialize() override;

	/// Wake up the view
	virtual bool doWakeUp() override;

private:
	/// Position of the view on the screen (in pixels)
	int m_x, m_y;
	/// Dimensions of the view on the screen (in pixels)
	int m_width, m_height;
	/// Whether the view window has a border
	bool m_isWindowBorderEnabled;

	/// Whether the next update will be the first time the view has been updated
	/// On the first update, the view window is setup.
	bool m_isFirstUpdate;
	/// Whether the settings have been changed and the window needs to be updated
	bool m_areWindowSettingsDirty;

	/// OSG view which performs the actual work involved in setting up and rendering to a window
	osg::ref_ptr<osgViewer::View> m_view;

	osg::ref_ptr<OsgTrackballZoomManipulator> m_manipulator;
	SurgSim::Math::Vector3d m_manipulatorPosition;
	SurgSim::Math::Vector3d m_manipulatorLookat;

	/// Indicate if a keyboard device is enabled
	bool m_keyboardEnabled;
	/// Indicate if a mouse device is enabled
	bool m_mouseEnabled;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGVIEW_H
