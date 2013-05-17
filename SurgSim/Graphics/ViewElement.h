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

#include <SurgSim/Framework/SceneElement.h>

namespace SurgSim
{

namespace Graphics
{

class Camera;
class View;

/// Basic SceneElement that wraps a View so that it can be added to the Scene.
///
/// A Scene needs at least one Graphics::View component for any visualization of Graphics:Actor objects to be shown.
class ViewElement : public Framework::SceneElement
{
public:
	/// Constructor
	/// \param	name	Name of the scene element
	/// \param	view	View component that provides the visualization of the graphics actors
	ViewElement(const std::string& name, std::shared_ptr<SurgSim::Graphics::View> view);

	/// Destructor
	virtual ~ViewElement();

	/// Sets the view component that provides the visualization of the graphics actors
	/// \return	True if it succeeds, false if it fails
	virtual bool setView(std::shared_ptr<View> view);

	/// Returns the view component that provides the visualization of the graphics actors
	std::shared_ptr<View> getView() const
	{
		return m_view;
	}

protected:
	/// Initializes the scene element
	/// \return True if it succeeds, false if it fails
	virtual bool doInitialize();

private:
	/// Wakes up the scene element
	/// \return True if it succeeds, false if it fails
	virtual bool doWakeUp();

	/// View component that provides the visualization of the graphics actors
	std::shared_ptr<View> m_view;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_VIEWELEMENT_H
