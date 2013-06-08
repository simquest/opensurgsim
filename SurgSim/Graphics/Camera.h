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

#ifndef SURGSIM_GRAPHICS_CAMERA_H
#define SURGSIM_GRAPHICS_CAMERA_H

#include <SurgSim/Graphics/Actor.h>

#include <SurgSim/Math/Matrix.h>

namespace SurgSim
{

namespace Graphics
{

class Group;

/// Base graphics camera class, which defines the basic interface for all graphics cameras.
///
/// A Graphics::Camera provides the viewpoint to visualize the Graphics::Group assigned to it.
///
/// To disable a camera: setVisible(false). To re-enable, setVisible(true).
/// A disabled (invisible) camera does not produce an image.
///
/// Graphics::Camera is used with Graphics::View to provide the visualization of the virtual scene to the user.
class Camera : public virtual Actor
{
public:
	/// Constructor
	/// \param	name	Name of the camera
	explicit Camera(const std::string& name) : Actor(name)
	{
	}

	/// Sets the group of actors that will be seen by this camera.
	/// Only the actors in this group will be rendered when this camera's view is rendered.
	/// \param	group	Group of actors
	/// \return	True if it succeeded, false if it failed
	virtual bool setGroup(std::shared_ptr<Group> group)
	{
		m_group = group;
		return true;
	}

	/// Gets the group of actors that will be seen by this camera.
	/// Only the actors in this group will be rendered when this camera's view is rendered.
	/// \return	Group of actors
	std::shared_ptr<Group> getGroup() const
	{
		return m_group;
	}

	/// Sets the pose of the camera
	/// \param	transform	Rigid transformation that describes the pose of the camera
	virtual void setPose(const SurgSim::Math::RigidTransform3d& transform) = 0;

	/// Gets the pose of the camera
	/// \return	Rigid transformation that describes the pose of the camera
	virtual const SurgSim::Math::RigidTransform3d& getPose() const = 0;

	/// Sets the view matrix of the camera
	/// \param	matrix	View matrix
	virtual void setViewMatrix(const SurgSim::Math::Matrix44d& matrix) = 0;

	/// Gets the view matrix of the camera
	/// \return	View matrix
	virtual const SurgSim::Math::Matrix44d& getViewMatrix() const = 0;

	/// Sets the projection matrix of the camera
	/// \param	matrix	Projection matrix
	virtual void setProjectionMatrix(const SurgSim::Math::Matrix44d& matrix) = 0;

	/// Gets the projection matrix of the camera
	/// \return	Projection matrix
	virtual const SurgSim::Math::Matrix44d& getProjectionMatrix() const = 0;

private:
	/// Group of actors that this camera sees
	/// Only the actors in this group will be rendered when this camera's view is rendered.
	std::shared_ptr<Group> m_group;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_CAMERA_H
