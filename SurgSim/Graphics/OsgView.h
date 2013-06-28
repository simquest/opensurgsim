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

#include <SurgSim/Graphics/View.h>

#include <osgViewer/Viewer>

namespace SurgSim
{

namespace Graphics
{

class OsgCamera;

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

	/// Set the position of this view
	/// \param	x,y	Position on the screen (in pixels)
	/// \return	True if it succeeded, false if it failed
	virtual bool setPosition(int x, int y);

	/// Get the position of this view
	/// \param[out]	x,y	Position on the screen (in pixels)
	virtual void getPosition(int* x, int* y) const;

	/// Set the dimensions of this view
	/// \param	width,height	Dimensions on the screen (in pixels)
	/// \return	True if it succeeded, false if it failed
	virtual bool setDimensions(int width, int height);

	/// Set the dimensions of this view
	/// \param[out]	width,height	Dimensions on the screen (in pixels)
	virtual void getDimensions(int* width, int* height) const;

	/// Sets whether the view window has a border
	/// \param	enabled	True to enable the border around the window; false for no border
	virtual void setWindowBorderEnabled(bool enabled);
	/// Returns whether the view window has a border
	/// \return	True to enable the border around the window; false for no border
	virtual bool isWindowBorderEnabled() const;

	/// Sets the camera which provides the viewpoint in the scene
	/// Only allows OsgCamera components, any other will not be set and it will return false.
	/// \param	camera	Camera whose image will be shown in this view
	/// \return	True if it succeeded, false if it failed
	virtual bool setCamera(std::shared_ptr<Camera> camera);

	/// Updates the view
	/// On the first update, the view window is setup.
	/// If the position or dimensions have changed, the window rectangle is updated.
	/// \param	dt	The time in seconds of the preceding timestep.
	virtual void update(double dt);

	/// Returns the OSG view which performs the actual work involved in setting up and rendering to a window
	osg::ref_ptr<osgViewer::View> getOsgView() const
	{
		return m_view;
	}

protected:
	/// Initialize the view
	/// \post The view's window is setup.
	virtual bool doInitialize();

	/// Wake up the view
	virtual bool doWakeUp();

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
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGVIEW_H
