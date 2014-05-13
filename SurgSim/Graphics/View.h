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

#ifndef SURGSIM_GRAPHICS_VIEW_H
#define SURGSIM_GRAPHICS_VIEW_H

#include "SurgSim/Framework/Component.h"

#include "SurgSim/Math/Vector.h"

#include <memory>

namespace SurgSim
{

namespace Graphics
{

class Camera;

/// Base graphics view class, which defines the basic interface for all graphics views.
///
/// A Graphics::View provides a visualization of the scene to the user.
///
/// A Graphics::Camera controls the viewpoint of this View.
class View : public SurgSim::Framework::Component
{
public:
	/// Constructor
	/// \param	name	Name of the view
	explicit View(const std::string& name);

	enum StereoMode
	{
		STEREO_MODE_NONE = -1,
		STEREO_MODE_QUAD_BUFFER,
		STEREO_MODE_ANAGLYPHIC,
		STEREO_MODE_HORIZONTAL_SPLIT,
		STEREO_MODE_VERTICAL_SPLIT,
		STEREO_MODE_LEFT_EYE,
		STEREO_MODE_RIGHT_EYE,
		STEREO_MODE_HORIZONTAL_INTERLACE,
		STEREO_MODE_VERTICAL_INTERLACE,
		STEREO_MODE_CHECKERBOARD,
		STEREO_MODE_COUNT
	};

	enum DisplayType
	{
		DISPLAY_TYPE_MONITOR,
		DISPLAY_TYPE_HMD,
		DISPLAY_TYPE_COUNT
	};

	/// Set the position of this view
	/// \param	x,y	Position on the screen (in pixels)
	/// \return	True if it succeeded, false if it failed
	virtual bool setPosition(int x, int y) = 0;

	/// Get the position of this view
	/// \param[out]	x,y	Position on the screen (in pixels)
	virtual void getPosition(int* x, int* y) const = 0;

	/// Set the dimensions of this view
	/// \param	width,height	Dimensions on the screen (in pixels)
	/// \return	True if it succeeded, false if it failed
	virtual bool setDimensions(int width, int height) = 0;

	/// Set the dimensions of this view
	/// \param[out]	width,height	Dimensions on the screen (in pixels)
	virtual void getDimensions(int* width, int* height) const = 0;

	/// Sets whether the view window has a border
	/// \param	enabled	True to enable the border around the window; false for no border
	virtual void setWindowBorderEnabled(bool enabled) = 0;

	/// Returns whether the view window has a border
	/// \return	True to enable the border around the window; false for no border
	virtual bool isWindowBorderEnabled() const = 0;

	/// Sets the camera which provides the viewpoint in the scene
	/// \param	camera	Camera whose image will be shown in this view
	/// \return	True if it succeeded, false if it failed
	virtual bool setCamera(std::shared_ptr<Camera> camera);

	/// Gets the camera which provides the viewpoint in the scene
	/// \return	camera	Camera whose image will be shown in this view
	std::shared_ptr<Camera> getCamera() const;

	/// Updates the view
	/// \param	dt	The time in seconds of the preceding timestep.
	virtual void update(double dt) = 0;

	virtual bool isStereo() = 0;

	/// Set the mode that this view should use for stereo display, see StereMode
	/// \param Value for StereoMode
	virtual void setStereoMode(StereoMode val) = 0;

	/// \return the Stereo
	virtual StereoMode getStereoMode() const = 0;

	virtual void setDisplayType(DisplayType type) = 0;

	virtual DisplayType getDisplayType() const = 0;

	virtual void setFullScreen(bool val) = 0;

	virtual bool isFullScreen() const = 0;

	virtual void setTargetScreen(int val) = 0;

	virtual int getTargetScreen() const = 0;

	virtual void setEyeSeparation(double val) = 0;

	virtual double getEyeSeparation() const =  0;

	virtual void setScreenDistance(double val) = 0;

	virtual double getScreenDistance() const = 0;

	virtual void setScreenWidth(double val) = 0;

	virtual double getScreenWidth() const = 0;

	virtual void setScreenHeight(double val) = 0;

	virtual double getScreenHeight() const = 0;


private:
	/// Camera whose image will be shown in this view
	std::shared_ptr<Camera> m_camera;

	virtual bool doInitialize() override;

};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_VIEW_H
