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

#ifndef SURGSIM_GRAPHICS_OSGVIEWELEMENT_H
#define SURGSIM_GRAPHICS_OSGVIEWELEMENT_H

#include <osg/ref_ptr>

#include "SurgSim/Graphics/ViewElement.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Input
{
class CommonDevice;
}

namespace Graphics
{

/// OSG-based implementation of graphics view element.
///
/// A Graphics::OsgViewElement creates and wraps a Graphics::OsgView so that it can be added to the Scene.
///
/// A Scene needs at least one Graphics::View component for any visualization of Graphics:Representation objects
/// to be shown.
class OsgViewElement : public Graphics::ViewElement
{
public:
	/// Constructor
	/// \param	name	Name of the scene element
	explicit OsgViewElement(const std::string& name);

	/// Destructor
	virtual ~OsgViewElement();

	/// Sets the view component that provides the visualization of the graphics representations
	/// Only allows OsgView components, any other will not be set and it will return false.
	/// \param view The view that should be used.
	/// \return	True if it succeeds, false if it fails.
	bool setView(std::shared_ptr<View> view) override;

	/// Enables a camera manipulator, implemented via a trackball, this is a temporary solution as it uses
	/// the OSG input events rather than reading from the OpenSurgSim input.
	/// \param val whether to enable the manipulator or not.
	void enableManipulator(bool val);

	/// As the camera is not accessible from here and as it cannot be controlled from the outside
	/// any more we let the user set the parameters from here.
	/// \param	position	The position of the camera.
	/// \param	lookat  	The location the camera looks at.
	void setManipulatorParameters(const SurgSim::Math::Vector3d& position, const SurgSim::Math::Vector3d& lookat);

	/// Return the keyboard to be used with this view.
	/// \return A keyboard device
	std::shared_ptr<SurgSim::Input::CommonDevice> getKeyboardDevice() override;
	/// Turn on/off the keyboard device to be used.
	/// \param val Indicate whether or not to use keyboard device
	void enableKeyboardDevice(bool val) override;

	/// Return the mouse to be used with this view.
	/// \return A mouse device
	std::shared_ptr<SurgSim::Input::CommonDevice> getMouseDevice() override;
	/// Turn on/off the mouse device to be used.
	/// \param val Indicate whether or not to use mouse device
	void enableMouseDevice(bool val) override;

private:
	/// Indicate if a keyboard device is enabled
	bool m_keyboardEnabled;
	/// Indicate if a mouse device is enabled
	bool m_mouseEnabled;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGVIEWELEMENT_H
