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

	bool setRenderGroup(std::shared_ptr<Group> group) override;

	bool setRenderGroups(const std::vector<std::shared_ptr<Group>>& groups) override;

	void setLocalActive(bool val) override;

	virtual SurgSim::Math::UnalignedMatrix44d getViewMatrix() const;

	virtual SurgSim::Math::UnalignedMatrix44d getInverseViewMatrix() const;

	void setProjectionMatrix(const SurgSim::Math::UnalignedMatrix44d& matrix) override;

	const SurgSim::Math::UnalignedMatrix44d& getProjectionMatrix() const override;

	SurgSim::Math::UnalignedMatrix44d getInverseProjectionMatrix() const override;

	void update(double dt) override;

	/// \return the OSG camera node
	osg::ref_ptr<osg::Camera> getOsgCamera() const;

	/// \return the OSG parent node for this object
	osg::ref_ptr<osg::Node> getOsgNode() const;

	bool setRenderTarget(std::shared_ptr<RenderTarget> renderTarget) override;

	std::shared_ptr<RenderTarget> getRenderTarget() const override;

	bool setMaterial(std::shared_ptr<SurgSim::Framework::Component> material) override;

	std::shared_ptr<Material> getMaterial() const override;

	void clearMaterial() override;

	void setRenderOrder(RenderOrder order, int value) override;

	void setAmbientColor(const SurgSim::Math::UnalignedVector4d& color) override;

	SurgSim::Math::UnalignedVector4d getAmbientColor() override;

	void setGenerateTangents(bool value) override;

	void setPerspectiveProjection(double fovy, double aspect, double near, double far) override;

	void setOrthogonalProjection(
		double left, double right,
		double bottom, double top,
		double near, double far) override;


	void setViewport(int x, int y, int width, int height) override;

	void getViewport(int* x, int* y, int* width, int* height) const override;

	void setViewportSize(std::array<double, 2> dimensions) override;

	std::array<double, 2> getViewportSize() const override;

	void setMainCamera(bool val) override;

	bool isMainCamera() override;

private:

	osg::ref_ptr<osg::Camera> m_camera;

	/// Projection matrix of the camera
	SurgSim::Math::UnalignedMatrix44d m_projectionMatrix;

	std::unordered_map<int, std::shared_ptr<Texture>> m_textureMap;
	std::shared_ptr<RenderTarget> m_renderTarget;

	/// Attach a specific texture to a specific BufferComponent, works for Depth and all the Colors.
	/// \param	buffer 	The BufferComponent enum.
	/// \param	texture	The specific texture to attach.
	void attachRenderTargetTexture(osg::Camera::BufferComponent buffer, std::shared_ptr<Texture> texture);

	/// Detach the current render target from the camera.
	void detachCurrentRenderTarget();

	/// Uniform to carry the view matrix
	std::shared_ptr<OsgUniform<SurgSim::Math::UnalignedMatrix44f>> m_viewMatrixUniform;

	/// Uniform to carry the inverse view matrix
	std::shared_ptr<OsgUniform<SurgSim::Math::UnalignedMatrix44f>> m_inverseViewMatrixUniform;

	/// Uniform to carry the ambient color
	std::shared_ptr<OsgUniform<SurgSim::Math::UnalignedVector4f>> m_ambientColorUniform;

	/// Value for ambient color
	SurgSim::Math::UnalignedVector4d m_ambientColor;

	bool m_isMainCamera;
};

};  // namespace Graphics
};  // namespace SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif  // SURGSIM_GRAPHICS_OSGCAMERA_H
