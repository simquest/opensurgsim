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

#include <unordered_map>

#include <SurgSim/Graphics/Camera.h>
#include <SurgSim/Graphics/OsgRepresentation.h>
#include <SurgSim/Graphics/Texture.h>

#include <osg/Camera>
#include <osg/Switch>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace SurgSim
{
namespace Graphics
{

class Material;
class Texture;
class RenderTarget;

/// OSG implementation of a graphics camera.
///
/// A Graphics::OsgCamera wraps a osg::Camera to provide camera functionality and a osg::Switch to allow enabling and
/// disabling of the camera.
class OsgCamera : public OsgRepresentation, public Camera
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
	virtual bool setGroup(std::shared_ptr<Group> group) override;

	/// Sets whether the camera is currently visible
	/// When the camera is invisible, it does not produce an image.
	/// \param	visible	True for visible, false for invisible
	virtual void setVisible(bool visible) override;

	/// Gets whether the camera is currently visible
	/// When the camera is invisible, it does not produce an image.
	/// \return	visible	True for visible, false for invisible
	virtual bool isVisible() const override;

	/// Sets the current pose of the camera
	/// The view matrix is set to the inverse of the transform.
	/// \param	transform	Rigid transformation that describes the current pose of the camera
	virtual void setPose(const SurgSim::Math::RigidTransform3d& transform) override;

	/// Gets the current pose of the camera
	/// The transform returned is the inverse of the view matrix.
	/// \return	Rigid transformation that describes the current pose of the camera
	virtual const SurgSim::Math::RigidTransform3d& getPose() const override;

	/// Sets the view matrix of the camera
	/// \param	matrix	View matrix
	virtual void setViewMatrix(const SurgSim::Math::Matrix44d& matrix) override;

	/// Gets the view matrix of the camera
	/// \return	View matrix
	virtual const SurgSim::Math::Matrix44d& getViewMatrix() const override;

	/// Sets the projection matrix of the camera
	/// \param	matrix	Projection matrix
	virtual void setProjectionMatrix(const SurgSim::Math::Matrix44d& matrix) override;

	/// Gets the projection matrix of the camera
	/// \return	Projection matrix
	virtual const SurgSim::Math::Matrix44d& getProjectionMatrix() const override;

	/// Updates the camera.
	/// \param	dt	The time in seconds of the preceding timestep.
	virtual void update(double dt) override;

	/// Returns the OSG camera node
	inline osg::ref_ptr<osg::Camera> getOsgCamera() const
	{
		return m_camera;
	}

	inline osg::ref_ptr<osg::Node> getOsgNode() const
	{
		return m_switch;
	}

	/// Sets RenderTarget for the current camera, enables the camera to render to off-screen textures.
	/// \param	renderTarget	The RenderTarget to be used.
	virtual bool setRenderTarget(std::shared_ptr<RenderTarget> renderTarget) override;

	/// Gets RenderTarget that is currently being used by the camera.
	/// \return	The RenderTarget.
	virtual std::shared_ptr<RenderTarget> getRenderTarget() const override;

	/// Sets a material on the group that has been attached to the camera.
	/// \param	material	The material.
	/// \return	true if it succeeds, false if there is no group or the material is not an OsgMaterial.
	virtual bool setMaterial(std::shared_ptr<Material> material) override;

	/// Gets the material if set.
	/// \return	The material.
	virtual std::shared_ptr<Material> getMaterial() const override;

	/// Clears the material from the attached group
	virtual void clearMaterial() override;

	/// Determine when this camera will render. The main camera will render at (RENDER_ORDER_IN_ORDER,0)
	/// In general all preprocessing should be done in RENDER_ORDER_PRE_ORDER, HUD Displaying usually
	/// at RENDER_ORDER_POST_ORDER. Overridden from Camera
	/// \param order The phase of rendering.
	/// \param value The index within the phase, the order between two cameras of the same phase and index is not
	/// 			 determined.
	virtual void setRenderOrder(RenderOrder order, int value) override;

private:

	osg::ref_ptr<osg::Camera> m_camera;
	osg::ref_ptr<osg::Group> m_materialProxy;

	/// Pose of the camera, which is the inverse of the view matrix
	SurgSim::Math::RigidTransform3d m_pose;
	/// View matrix of the camera
	SurgSim::Math::Matrix44d m_viewMatrix;
	/// Projection matrix of the camera
	SurgSim::Math::Matrix44d m_projectionMatrix;

	std::unordered_map<int, std::shared_ptr<Texture>> m_textureMap;
	std::shared_ptr<RenderTarget> m_renderTarget;

	/// Attach a specific texture to a specific BufferComponent, works for Depth and all the Colors.
	/// \param	buffer 	The BufferComponent enum.
	/// \param	texture	The specific texture to attach.
	void attachRenderTargetTexture(osg::Camera::BufferComponent buffer, std::shared_ptr<Texture> texture);

	/// Detach the current render target from the camera.
	void detachCurrentRenderTarget();

};

};  // namespace Graphics
};  // namespace SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif  // SURGSIM_GRAPHICS_OSGCAMERA_H
