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
#include <array>

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
	/// \param	position	Position on the screen (in pixels)
	/// \return	True if it succeeded, false if it failed
	virtual void setPosition(const std::array<int, 2>& position) = 0;

	/// Get the position of this view
	/// \return Position on the screen (in pixels)
	virtual std::array<int, 2> getPosition() const = 0;

	/// Set the dimensions of this view
	/// \param	dimensions Dimensions on the screen (in pixels)
	virtual void setDimensions(const std::array<int, 2>& dimensions) = 0;

	/// Get the dimensions of this view
	/// \return Dimensions on the screen (in pixels)
	virtual std::array<int, 2> getDimensions() const = 0;

	/// Set the dimensions of this view in doubles
	/// \param	dimensions Dimensions on the screen (in pixels)
	virtual void setDimensionsDouble(const std::array<double, 2>& dimensions) = 0;

	/// Get the dimensions of this view in doubles
	/// \return Dimensions on the screen (in pixels)
	virtual std::array<double, 2> getDimensionsDouble() const = 0;

	/// Sets whether the view window has a border
	/// \param	enabled	True to enable the border around the window; false for no border
	virtual void setWindowBorderEnabled(bool enabled) = 0;

	/// Returns whether the view window has a border
	/// \return	True to enable the border around the window; false for no border
	virtual bool isWindowBorderEnabled() const = 0;

	/// Sets the camera which provides the viewpoint in the scene
	/// \param	camera	Camera whose image will be shown in this view
	/// \return	True if it succeeded, false if it failed
	virtual void setCamera(std::shared_ptr<SurgSim::Framework::Component> camera);

	/// Gets the camera which provides the viewpoint in the scene
	/// \return	camera	Camera whose image will be shown in this view
	std::shared_ptr<Camera> getCamera() const;

	/// Updates the view
	/// \param	dt	The time in seconds of the preceding timestep.
	virtual void update(double dt) = 0;

	/// \return true if the display is set to render a stereo view, or is currently rendering in stereo.
	virtual bool isStereo() const;

	/// Set the mode that this view should use for stereo display, see StereMode for all the modes.
	/// \throws SurgSim::Framework::AssertionFailure if used after initialize has been called.
	/// \param val The actual mode.
	virtual void setStereoMode(int val);

	/// \return What kind of stereo rendering is being used for this view.
	int getStereoMode() const;

	/// Set the kind of display.
	/// \throws SurgSim::Framework::AssertionFailure if used after initialize has been called
	/// \param type The type of display
	void setDisplayType(int type);

	/// \return The type of display that the view is on
	int getDisplayType() const;

	/// Request the display to use the whole screen.
	/// \throws SurgSim::Framework::AssertionFailure if used after initialize has been called.
	/// \param val If true the display will use up the whole screen, ignoring the dimension and location settings
	void setFullScreen(bool val);

	/// \return true if the display is set to use the whole screen, or is currently using the whole screen.
	bool isFullScreen() const;

	/// Request a certain screen to be used for this view.
	/// \throws SurgSim::Framework::AssertionFailure if used after initialize has been called.
	/// \param val The number of the screen (base 0) that should be used for this view.
	void setTargetScreen(int val);

	/// \return The number of the screen that this view is on.
	int getTargetScreen() const;

	/// Set the distance between the users eyes, this is necessary to calculate the correct projection matrices
	/// for stereo rendering.
	/// \throws SurgSim::Framework::AssertionFailure if used after initialize has been called.
	/// \note this is only used when rendering stereo.
	/// \param val The distance between the eyes in m.
	void setEyeSeparation(double val);

	/// \return The current distance between the eye points in m.
	double getEyeSeparation() const;

	/// Set the distance of the user from the screen, this is necessary to calculate the correct projection matrices
	/// for stereo rendering.
	/// \throws SurgSim::Framework::AssertionFailure if used after initialize has been called.
	/// \note this is only used when rendering stereo.
	/// \param val The distance from the user to the screen in m.
	void setScreenDistance(double val);

	/// \return The current distance from user to screen in m.
	double getScreenDistance() const;

	/// Set the width of the screen, this is necessary to calculate the correct projection matrices
	/// for stereo rendering.
	/// \throws SurgSim::Framework::AssertionFailure if used after initialize has been called.
	/// \note this is only used when rendering stereo.
	/// \param val The width of the screen used, in m.
	void setScreenWidth(double val);

	/// \return The currently used width of the screen in m.
	double getScreenWidth() const;


	/// Set the height of the screen, this is necessary to calculate the correct projection matrices
	/// for stereo rendering.
	/// \throws SurgSim::Framework::AssertionFailure if used after initialize has been called.
	/// \note this is only used when rendering stereo.
	/// \param val The height of the screen used, in m.
	void setScreenHeight(double val);

	/// \return The currently used height of the screen in m.
	double getScreenHeight() const;

private:

	bool doInitialize() override;

	virtual int doSetTargetScreen(int val) = 0;

	/// Camera whose image will be shown in this view
	std::shared_ptr<Camera> m_camera;

	int m_stereoMode; ///< The stereo mode, that is being used.
	int m_displayType; ///< The requested display type.
	int m_targetScreen; ///< Index of the screen to be used
	bool m_isFullscreen; ///< Whether to go fullscreen
	double m_eyeSeparation; ///< Distance between eypoints in m.
	double m_screenDistance; ///< Distance from user to screen in m.
	double m_screenWidth; ///< Width of screen in m.
	double m_screenHeight; ///< Height of screen in m.

};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_VIEW_H
