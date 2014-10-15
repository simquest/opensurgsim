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

#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Graphics/Camera.h"
#include "SurgSim/Graphics/OsgRepresentation.h"
#include "SurgSim/Graphics/Texture.h"


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

template <class T>
class OsgUniform;

SURGSIM_STATIC_REGISTRATION(OsgCamera);

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

	SURGSIM_CLASSNAME(SurgSim::Graphics::OsgCamera);

	virtual bool setRenderGroup(std::shared_ptr<Group> group) override;

	virtual void setLocalActive(bool val) override;

	virtual SurgSim::Math::Matrix44d getViewMatrix() const;

	virtual SurgSim::Math::Matrix44d getInverseViewMatrix() const;

	virtual void setProjectionMatrix(const SurgSim::Math::Matrix44d& matrix) override;

	virtual const SurgSim::Math::Matrix44d& getProjectionMatrix() const override;

	virtual void update(double dt) override;

	/// \return the OSG camera node
	osg::ref_ptr<osg::Camera> getOsgCamera() const;

	/// \return the OSG parent node for this object
	osg::ref_ptr<osg::Node> getOsgNode() const;

	virtual bool setRenderTarget(std::shared_ptr<RenderTarget> renderTarget) override;

	virtual std::shared_ptr<RenderTarget> getRenderTarget() const override;

	virtual bool setMaterial(std::shared_ptr<Material> material) override;

	virtual std::shared_ptr<Material> getMaterial() const override;

	virtual void clearMaterial() override;

	virtual void setRenderOrder(RenderOrder order, int value) override;

	virtual void setAmbientColor(const SurgSim::Math::Vector4d& color) override;

	virtual SurgSim::Math::Vector4d getAmbientColor() override;

private:

	osg::ref_ptr<osg::Camera> m_camera;
	osg::ref_ptr<osg::Group> m_materialProxy;

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

	/// Uniform to carry the view matrix
	std::shared_ptr<OsgUniform<SurgSim::Math::Matrix44f>> m_viewMatrixUniform;

	/// Uniform to carry the inverse view matrix
	std::shared_ptr<OsgUniform<SurgSim::Math::Matrix44f>> m_inverseViewMatrixUniform;

	/// Uniform to carry the ambient color
	std::shared_ptr<OsgUniform<SurgSim::Math::Vector4f>> m_ambientColorUniform;

	/// Value for ambient color
	SurgSim::Math::Vector4d m_ambientColor;


};

};  // namespace Graphics
};  // namespace SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif  // SURGSIM_GRAPHICS_OSGCAMERA_H
