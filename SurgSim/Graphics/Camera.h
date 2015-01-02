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

#include "SurgSim/Graphics/Representation.h"

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Graphics
{

class Group;
class Texture;
class RenderTarget;

/// Base graphics camera class, which defines the basic interface for all graphics cameras.
///
/// A Graphics::Camera provides the viewpoint to visualize the Graphics::Group assigned to it.
///
/// To disable a camera: setLocalActive(false). To re-enable, setLocalActive(true).
/// A disabled (invisible) camera does not produce an image.
///
/// Graphics::Camera is used with Graphics::View to provide the visualization of the virtual scene to the user.
/// Cameras refer to a group that contain all the elements that they render, they may also parts of other group that
/// determine whether they are rendered.
/// It should provide the following Uniforms:
/// \code
/// uniform mat4 viewMatrix;
/// uniform mat4 inverseViewMatrix;
/// \endcode
class Camera : public virtual Representation
{
public:

	enum RenderOrder
	{
		RENDER_ORDER_PRE_RENDER = 0,
		RENDER_ORDER_IN_ORDER,
		RENDER_ORDER_POST_RENDER,
		RENDER_ORDER_COUNT
	};

	/// Constructor
	/// \param	name	Name of the camera
	explicit Camera(const std::string& name);

	/// Set the group reference that this camera wants to use as the group for rendering. Objects that, reference
	/// the same group will be rendered by this camera. The manager will do the actual creation of the group.
	/// \param name Name of the group to be used for rendering
	void setRenderGroupReference(const std::string& name);

	/// Gets the name of the rendergroup used for rendering
	/// \return The name of the group to be used for rendering
	std::string getRenderGroupReference() const;

	/// Sets the group of representations that will be seen by this camera.
	/// Only the representations in this group will be rendered when this camera's view is rendered.
	/// \note The camera can not be part of the group that it is rendering
	/// \param	group	Group of representations
	/// \return	True if it succeeded, false if it failed
	virtual bool setRenderGroup(std::shared_ptr<Group> group);

	/// Gets the group of representations that will be seen by this camera.
	/// Only the representations in this group will be rendered when this camera's view is rendered.
	/// \return	Group of representations
	std::shared_ptr<Group> getRenderGroup() const;

	/// Gets the view matrix of the camera
	/// \return	View matrix
	virtual SurgSim::Math::Matrix44d getViewMatrix() const = 0;

	/// Gets the inverse view matrix of the camera
	/// \return	Inverse view matrix
	virtual SurgSim::Math::Matrix44d getInverseViewMatrix() const = 0;

	/// Sets the projection matrix of the camera
	/// \param	matrix	Projection matrix
	virtual void setProjectionMatrix(const SurgSim::Math::Matrix44d& matrix) = 0;

	/// Gets the projection matrix of the camera
	/// \return	Projection matrix
	virtual const SurgSim::Math::Matrix44d& getProjectionMatrix() const = 0;

	/// Sets RenderTarget for the current camera, enables the camera to render to off-screen textures.
	/// \param	renderTarget	The render target.
	virtual bool setRenderTarget(std::shared_ptr<RenderTarget> renderTarget) = 0;

	/// Gets RenderTarget that is currently being used by the camera.
	/// \return	The RenderTarget.
	virtual std::shared_ptr<RenderTarget> getRenderTarget() const = 0;

	/// Determine when this camera will render. The main camera will render at (RENDER_ORDER_IN_ORDER,0)
	/// In general all preprocessing should be done in RENDER_ORDER_PRE_ORDER, HUD Displaying usually
	/// at RENDER_ORDER_POST_ORDER
	/// \param order The phase of rendering.
	/// \param value The index within the phase, the order between two cameras of the same phase and index is not
	/// 			 determined.
	virtual void setRenderOrder(RenderOrder order, int value) = 0;

	bool addGroupReference(const std::string& name) override;

	/// Sets a value for the ambient lighting term, this can add light to the scene when there is no lighting
	/// \param color value for the light that should get added to the scene
	virtual void setAmbientColor(const SurgSim::Math::Vector4d& color) = 0;

	/// \return the ambient light that gets added to the scene
	virtual SurgSim::Math::Vector4d getAmbientColor() = 0;


private:

	bool doInitialize() override;

	/// Group of representations that this camera sees
	/// Only the representations in this group will be rendered when this camera's view is rendered.
	std::shared_ptr<Group> m_group;

	/// The name of the group that the camera wants to use for rendering, the graphics manager will actually assign
	/// this group
	std::string m_renderGroupReference;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_CAMERA_H
