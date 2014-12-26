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

#ifndef SURGSIM_GRAPHICS_VIEWELEMENT_H
#define SURGSIM_GRAPHICS_VIEWELEMENT_H

#include "SurgSim/Framework/BasicSceneElement.h"

namespace SurgSim
{

namespace Input
{
class CommonDevice;
}

namespace Graphics
{

class Camera;
class View;

/// Basic SceneElement that wraps a View so that it can be added to the Scene.
///
/// A Scene needs at least one Graphics::View component for any visualization of Graphics:Representation objects
/// to be shown. A view needs a camera to do its' rendering, this component can connect all the pieces correctly.
class ViewElement : public Framework::BasicSceneElement
{
public:
	/// Constructor
	/// \param	name	Name of the scene element
	explicit ViewElement(const std::string& name);

	/// Destructor
	virtual ~ViewElement();

	/// Sets the view component that provides the visualization of the graphics representations
	/// \return	True if setView() succeeds; Otherwise, false.
	virtual bool setView(std::shared_ptr<View> view);

	/// Returns the view component that provides the visualization of the graphics representations
	/// \return A shared_ptr pointing to the View component
	std::shared_ptr<View> getView();

	/// Sets the camera for the view in this sceneelement
	/// \param camera The camera to be used with the view
	void setCamera(std::shared_ptr<Camera> camera);

	/// Get the camera for the view in this sceneelement.
	/// \return The camera used in the view of this sceneelement.
	std::shared_ptr<Camera> getCamera();

	/// Return the keyboard to be used with this view.
	/// \return A keyboard device
	virtual std::shared_ptr<SurgSim::Input::CommonDevice> getKeyboardDevice() = 0;

	/// Turn on/off the keyboard device to be used.
	/// \param val Indicate whether or not to use keyboard device
	virtual	void enableKeyboardDevice(bool val) = 0;

	/// Return the mouse to be used with this view.
	/// \return A mouse device
	virtual std::shared_ptr<SurgSim::Input::CommonDevice> getMouseDevice() = 0;

	/// Turn on/off the mouse device to be used.
	/// \param val Indicate whether or not to use mouse device
	virtual	void enableMouseDevice(bool val) = 0;

protected:
	/// Initializes the scene element
	/// \return True if it succeeds, false if it fails
	bool doInitialize() override;

private:
	/// View component that provides the visualization of the graphics representations
	std::shared_ptr<View> m_view;

	/// Camera component connected to the view
	std::shared_ptr<Camera> m_camera;
};

};  // namespace Graphics
};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_VIEWELEMENT_H
