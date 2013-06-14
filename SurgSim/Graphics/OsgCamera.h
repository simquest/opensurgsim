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

#ifndef SURGSIM_GRAPHICS_OSGCAMERA_H
#define SURGSIM_GRAPHICS_OSGCAMERA_H

#include <SurgSim/Graphics/Camera.h>
#include <SurgSim/Graphics/OsgRepresentation.h>

#include <osg/Camera>
#include <osg/Switch>

namespace SurgSim
{

namespace Graphics
{

/// OSG implementation of a graphics camera.
///
/// A Graphics::OsgCamera wraps a osg::Camera to provide camera functionality and a osg::Switch to allow enabling and
/// disabling of the camera.
class OsgCamera : public Camera, public OsgRepresentation
{
public:
	/// Constructor
	/// \param	name	Name of the camera
	/// The view matrix is initialized with eye at (0, 0, 0), center at (0, 0, -1), and up (0, 1, 0).
	/// The projection matrix is initialized to a perspective matrix with FOV Y of 45 deg, Aspect Ratio of 1.0,
	/// Z Near of 0.01, and Z Far of 10.0.
	explicit OsgCamera(const std::string& name);

	/// Sets the group of representations that will be seen by this camera.
	/// Only the representations in this group will be rendered when this camera's view is rendered.
	/// \param	group	Group of representations
	/// \return	True if it succeeded, false if it failed
	virtual bool setGroup(std::shared_ptr<Group> group);

	/// Sets whether the camera is currently visible
	/// When the camera is invisible, it does not produce an image.
	/// \param	visible	True for visible, false for invisible
	virtual void setVisible(bool visible);

	/// Gets whether the camera is currently visible
	/// When the camera is invisible, it does not produce an image.
	/// \return	visible	True for visible, false for invisible
	virtual bool isVisible() const;

	/// Sets the current pose of the camera
	/// The view matrix is set to the inverse of the transform.
	/// \param	transform	Rigid transformation that describes the current pose of the camera
	virtual void setPose(const SurgSim::Math::RigidTransform3d& transform);

	/// Gets the current pose of the camera
	/// The transform returned is the inverse of the view matrix.
	/// \return	Rigid transformation that describes the current pose of the camera
	virtual const SurgSim::Math::RigidTransform3d& getPose() const;

	/// Sets the view matrix of the camera
	/// \param	matrix	View matrix
	virtual void setViewMatrix(const SurgSim::Math::Matrix44d& matrix);

	/// Gets the view matrix of the camera
	/// \return	View matrix
	virtual const SurgSim::Math::Matrix44d& getViewMatrix() const;

	/// Sets the projection matrix of the camera
	/// \param	matrix	Projection matrix
	virtual void setProjectionMatrix(const SurgSim::Math::Matrix44d& matrix);

	/// Gets the projection matrix of the camera
	/// \return	Projection matrix
	virtual const SurgSim::Math::Matrix44d& getProjectionMatrix() const;

	/// Updates the camera.
	/// \param	dt	The time in seconds of the preceding timestep.
	virtual void update(double dt);

	/// Returns the OSG camera node
	osg::ref_ptr<osg::Camera> getOsgCamera() const
	{
		return m_camera;
	}

private:
	/// OSG switch to allow enabling and disabling of the camera
	osg::ref_ptr<osg::Switch> m_switch;
	/// OSG camera node
	osg::ref_ptr<osg::Camera> m_camera;

	/// Pose of the camera, which is the inverse of the view matrix
	SurgSim::Math::RigidTransform3d m_pose;
	/// View matrix of the camera
	SurgSim::Math::Matrix44d m_viewMatrix;
	/// Projection matrix of the camera
	SurgSim::Math::Matrix44d m_projectionMatrix;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGCAMERA_H
