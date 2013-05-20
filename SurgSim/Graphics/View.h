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

#include <SurgSim/Framework/Component.h>

#include <SurgSim/Math/Vector.h>

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
	explicit View(const std::string& name) : SurgSim::Framework::Component(name)
	{
	}

	/// Set the position of this view
	/// \param	x,y	Position on the screen (in pixels)
	/// \return	True if it succeeded, false if it failed
	virtual bool setPosition(int x, int y) = 0;

	/// Get the position of this view
	/// \param[out]	x,y	Position on the screen (in pixels)
	virtual void getPosition(int* x, int* y) = 0;

	/// Set the dimensions of this view
	/// \param	width,height	Dimensions on the screen (in pixels)
	/// \return	True if it succeeded, false if it failed
	virtual bool setDimensions(int width, int height) = 0;

	/// Set the dimensions of this view
	/// \param[out]	width,height	Dimensions on the screen (in pixels)
	virtual void getDimensions(int* width, int* height) = 0;

	/// Sets the camera which provides the viewpoint in the scene
	/// \param	camera	Camera whose image will be shown in this view
	virtual void setCamera(std::shared_ptr<Camera> camera)
	{
		m_camera = camera;
	}
	/// Gets the camera which provides the viewpoint in the scene
	/// \return	camera	Camera whose image will be shown in this view
	std::shared_ptr<Camera> getCamera() const
	{
		return m_camera;
	}

	/// Updates the view
	/// \param	dt	The time in seconds of the preceding timestep.
	virtual void update(double dt) = 0;

private:
	/// Camera whose image will be shown in this view
	std::shared_ptr<Camera> m_camera;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_VIEW_H
