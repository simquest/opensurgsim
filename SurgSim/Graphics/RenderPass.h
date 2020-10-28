// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_GRAPHICS_RENDERPASS_H
#define SURGSIM_GRAPHICS_RENDERPASS_H

#include <memory>

#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/Camera.h"

namespace SurgSim
{

namespace Framework
{

class Component;

}
namespace Graphics
{

class Group;
class Material;
class RenderTarget;
class ScreenSpaceQuadRepresentation;
class Texture;
class View;


/// Encapsulation of all the components necessary needed to implement a full renderpass, this SceneElement contains
/// a Camera and Group, it can also take a Material (for shaders and uniforms) and a RenderTarget for textures that
/// are used as the output for the camera.
/// Other components do not need to be added to the pass explicitly, they only need to refer to the passes name in
/// their respective groupReferences to be added to the render pass. The passes attributes should all be set up through
/// uniforms in the material.
class RenderPass : public SurgSim::Framework::SceneElement
{
public:

	/// Constructor
	/// \param name The name for this SceneElement
	explicit RenderPass(const std::string& name);
	~RenderPass();

	/// Executes the initialize operation.
	/// \return	true if it succeeds, false if it fails.
	bool doInitialize() override;

	/// Sets render target for the camera, this abstracts the textures that are being used for rendering into.
	/// \param	target	The rendertarget structure.
	/// \return true if the target was successfully set
	bool setRenderTarget(std::shared_ptr<RenderTarget> target);

	/// Gets render target that is being used in this pass.
	/// \return	The render target that should be used.
	std::shared_ptr<RenderTarget> getRenderTarget();

	/// Sets render order.
	/// \param	order	The general render stage for this pass.
	/// \param	value	An order value for this pass, lower means earlier.
	virtual void setRenderOrder(SurgSim::Graphics::Camera::RenderOrder order, int value);

	/// Gets the camera.
	/// \return	The camera.
	std::shared_ptr<Camera> getCamera();

	/// Sets the material used for rendering.
	/// \param	material	The material.
	/// \return	true if it succeeds, false if it fails.
	bool setMaterial(std::shared_ptr<SurgSim::Framework::Component> material);

	/// Gets the current material.
	/// \return	The material.
	std::shared_ptr<Material> getMaterial();

	/// Shows a quad on the screen with the texture used as the color target for this pass.
	/// \param	x,y	  	The x and y coordinates on the screen.
	/// \param	width,height 	The width and height on the scree.
	void showColorTarget(int x, int y, int width, int height);

	/// Hides the color target display.
	void hideColorTarget();

	/// Shows a quad on the screen with the texture used as the depth target for this pass.
	/// \param	x,y	  	The x and y coordinates on the screen.
	/// \param	width,height 	The width and height on the screen.
	void showDepthTarget(int x, int y, int width, int height);

	/// Hides the depth target display.
	void hideDepthTarget();

private:


	std::shared_ptr<Camera> m_camera;				///< The camera used for the pass
	std::shared_ptr<Group> m_group;					///< The groupd used for the pass
	std::shared_ptr<RenderTarget> m_renderTarget;	///< The camera's rendertarget
	std::shared_ptr<Material> m_material;			///< The material, attached to the camera

	std::shared_ptr<ScreenSpaceQuadRepresentation> m_debugColor;
	std::shared_ptr<ScreenSpaceQuadRepresentation> m_debugDepth;

	/// Utility function to build a debug quad.
	/// \param	name   	The name for the component.
	/// \param	texture	The texture for redering.
	/// \return	a constructed quads.
	std::shared_ptr<ScreenSpaceQuadRepresentation> buildDebugQuad(
		const std::string& name,
		std::shared_ptr<Texture> texture);
};

}; // Graphics
}; // SurgSim

#endif
