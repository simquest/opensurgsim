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

#include "SurgSim/Graphics/OsgCamera.h"

#include "SurgSim/Graphics/Manager.h"
#include "SurgSim/Graphics/Material.h"
#include "SurgSim/Graphics/OsgGroup.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgMatrixConversions.h"
#include "SurgSim/Graphics/OsgQuaternionConversions.h"
#include "SurgSim/Graphics/OsgRenderTarget.h"
#include "SurgSim/Graphics/OsgUniform.h"
#include "SurgSim/Graphics/OsgVectorConversions.h"
#include "SurgSim/Math/Matrix.h"

#include <osgUtil/CullVisitor>

using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::Matrix44f;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Vector4f;

namespace
{
const osg::Camera::BufferComponent ColorBufferEnums[16] =
{
	osg::Camera::COLOR_BUFFER0,
	osg::Camera::COLOR_BUFFER1,
	osg::Camera::COLOR_BUFFER2,
	osg::Camera::COLOR_BUFFER3,
	osg::Camera::COLOR_BUFFER4,
	osg::Camera::COLOR_BUFFER5,
	osg::Camera::COLOR_BUFFER6,
	osg::Camera::COLOR_BUFFER7,
	osg::Camera::COLOR_BUFFER8,
	osg::Camera::COLOR_BUFFER9,
	osg::Camera::COLOR_BUFFER10,
	osg::Camera::COLOR_BUFFER11,
	osg::Camera::COLOR_BUFFER12,
	osg::Camera::COLOR_BUFFER13,
	osg::Camera::COLOR_BUFFER14,
	osg::Camera::COLOR_BUFFER15
};

const osg::Camera::RenderOrder RenderOrderEnums[3] =
{
	osg::Camera::PRE_RENDER,
	osg::Camera::NESTED_RENDER,
	osg::Camera::POST_RENDER
};

/// Update the main camera uniforms to reflect the state of the rendering camera, this is used when rendering
/// stereo via osg, the mainCamera.* uniforms will carry the values of the currently rendering camera, i.e. the left
/// and the right view camera.
class UniformUpdater : public osg::NodeCallback
{
public:
	UniformUpdater(osg::Uniform* projection, osg::Uniform* inverseProjection,
				   osg::Uniform* view,	osg::Uniform* inverseView) :
		m_projection(projection),
		m_inverseProjection(inverseProjection),
		m_view(view),
		m_inverseView(inverseView)
	{

	}

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nodeVisitor)
	{
		osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nodeVisitor);
		if (cv != nullptr)
		{
			auto camera = cv->getCurrentCamera();
			const auto& projection = camera->getProjectionMatrix();
			m_projection->set(projection);
			m_inverseProjection->set(osg::Matrix::inverse(projection));
			m_view->set(camera->getViewMatrix());
			m_inverseView->set(camera->getInverseViewMatrix());
		}
		traverse(node, nodeVisitor);
	}

private:
	osg::ref_ptr<osg::Uniform> m_projection;
	osg::ref_ptr<osg::Uniform> m_inverseProjection;
	osg::ref_ptr<osg::Uniform> m_view;
	osg::ref_ptr<osg::Uniform> m_inverseView;
};

osg::Uniform* addMatrixUniform(osg::Node* node, const std::string& name)
{
	auto uniform = new osg::Uniform;

	uniform->setName(name);
	uniform->setType(osg::Uniform::FLOAT_MAT4);

	osg::Matrix matrix;
	uniform->set(matrix);

	node->getOrCreateStateSet()->addUniform(uniform);

	return uniform;
}

osg::Switch* createUniformUpdateNode(long mask) // NOLINT
{
	auto node = new osg::Switch;

	auto projection = addMatrixUniform(node, "mainCamera.projectionMatrix");
	auto inverseProjection = addMatrixUniform(node, "mainCamera.inverseProjectionMatrix");
	auto view = addMatrixUniform(node, "mainCamera.viewMatrix");
	auto inverseView = addMatrixUniform(node, "mainCamera.inverseViewMatrix");

	auto callback = new UniformUpdater(projection, inverseProjection, view, inverseView);
	node->addCullCallback(callback);
	node->setNodeMask(mask);

	return node;
}

const long CullMask = 0xfffffff1; // NOLINT
const long CullMaskLeft = 0x1; // NOLINT
const long CullMaskRight = 0x2; // NOLINT

};


namespace SurgSim
{
namespace Graphics
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Graphics::OsgCamera, OsgCamera);

OsgCamera::OsgCamera(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	Camera(name),
	m_camera(new osg::Camera()),
	m_viewMatrixUniform(std::make_shared<OsgUniform<Matrix44f>>("viewMatrix")),
	m_inverseViewMatrixUniform(std::make_shared<OsgUniform<Matrix44f>>("inverseViewMatrix")),
	m_ambientColorUniform(std::make_shared<OsgUniform<Vector4f>>("ambientColor")),
	m_isMainCamera(false)
{
	m_switch->removeChildren(0, m_switch->getNumChildren());
	m_camera->setName(name + " Camera");

	m_switch->addChild(m_camera);
	m_camera->addChild(m_materialProxy);
	m_materialProxy->setName(name + " MaterialProxy");

	m_camera->setViewMatrix(toOsg(getLocalPose().inverse().matrix()));
	setPerspectiveProjection(45.0, 1.0, 0.01, 10.0);

	m_camera->setComputeNearFarMode(osgUtil::CullVisitor::DO_NOT_COMPUTE_NEAR_FAR);

	// Hopefully this will ignore the left and right nodes when rendering mono and
	// pick the appropriate node when rendering stereo
	m_camera->setCullMask(CullMask);
	m_camera->setCullMaskLeft(CullMaskLeft);
	m_camera->setCullMaskRight(CullMaskRight);

	/// Update storage of view and projection matrices
	m_projectionMatrix = fromOsg(m_camera->getProjectionMatrix());

	// Set up uniforms
	osg::ref_ptr<osg::StateSet> state = m_camera->getOrCreateStateSet();
	m_viewMatrixUniform->addToStateSet(state);
	m_inverseViewMatrixUniform->addToStateSet(state);
	m_ambientColorUniform->addToStateSet(state);

	setAmbientColor(Vector4d(0.0, 0.0, 0.0, 0.0));

	// We will want this in all cases
	m_camera->getOrCreateStateSet()->setMode(GL_TEXTURE_CUBE_MAP_SEAMLESS, osg::StateAttribute::ON);
}

bool OsgCamera::setRenderGroup(std::shared_ptr<SurgSim::Graphics::Group> group)
{

	SURGSIM_ASSERT(group->getName() == Camera::getRenderGroupReference())
			<< "Trying to set the wrong group in the camera. getRenderGroupName() returns <"
			<< Camera::getRenderGroupReference() << "> group->getName() is <" << group->getName() << ">.";

	std::shared_ptr<OsgGroup> osgGroup = std::dynamic_pointer_cast<OsgGroup>(group);
	if (osgGroup && SurgSim::Graphics::Camera::setRenderGroup(group))
	{
		m_materialProxy->removeChildren(0, m_camera->getNumChildren());  /// Remove any previous group
		m_materialProxy->addChild(osgGroup->getOsgGroup());
		return true;
	}
	else
	{
		return false;
	}
}

void OsgCamera::setLocalActive(bool val)
{
	Component::setLocalActive(val);
	m_switch->setChildValue(m_camera, isActive());
}

SurgSim::Math::Matrix44d OsgCamera::getViewMatrix() const
{
	return getPose().matrix().inverse();
}

void OsgCamera::setProjectionMatrix(const SurgSim::Math::Matrix44d& matrix)
{
	m_projectionMatrix = matrix;
	m_camera->setProjectionMatrix(toOsg(matrix));
}

const SurgSim::Math::Matrix44d& OsgCamera::getProjectionMatrix() const
{
	return m_projectionMatrix;
}

SurgSim::Math::Matrix44d OsgCamera::getInverseProjectionMatrix() const
{
	return m_projectionMatrix.inverse();
}

void OsgCamera::update(double dt)
{
	setVisible(isActive());

	if (isActive())
	{
		// HS-2014-may-05 There is an issue between setting the projection matrix in the constructor and the
		// instantiation of the viewer with the view port that may change the matrix inbetween, for now ... update
		// every frame
		// #workaround
		// m_projectionMatrix = fromOsg(m_camera->getProjectionMatrix());

		auto viewMatrix = getViewMatrix();
		auto floatMatrix = viewMatrix.cast<float>();
		m_camera->setViewMatrix(toOsg(viewMatrix));
		m_viewMatrixUniform->set(floatMatrix);
		m_inverseViewMatrixUniform->set(floatMatrix.inverse());
	}
}

bool OsgCamera::setRenderTarget(std::shared_ptr<RenderTarget> renderTarget)
{
	bool result = false;

	// Check for correct dynamic type
	auto osg2dTarget = std::dynamic_pointer_cast<OsgRenderTarget2d>(renderTarget);
	auto osgRectTarget = std::dynamic_pointer_cast<OsgRenderTargetRectangle>(renderTarget);

	if (osg2dTarget != nullptr || osgRectTarget != nullptr)
	{
		if (m_renderTarget == nullptr)
		{
			detachCurrentRenderTarget();
		}

		int width, height;
		renderTarget->getSize(&width, &height);
		m_camera->setViewport(0, 0, width, height);

		attachRenderTargetTexture(osg::Camera::DEPTH_BUFFER, renderTarget->getDepthTarget());

		// OSG has 16 COLOR_BUFFER objects
		for (int i = 0; i < 16; ++i)
		{
			attachRenderTargetTexture(ColorBufferEnums[i], renderTarget->getColorTarget(i));
		}

		m_camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT, osg::Camera::PIXEL_BUFFER);
		m_camera->setRenderOrder(osg::Camera::PRE_RENDER);
		m_camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
		m_renderTarget = renderTarget;
		result = true;
	}
	else if (renderTarget == nullptr && m_renderTarget != nullptr)
	{
		detachCurrentRenderTarget();
		result = true;
	}

	return result;
}

std::shared_ptr<RenderTarget> OsgCamera::getRenderTarget() const
{
	return m_renderTarget;
}


bool OsgCamera::setMaterial(std::shared_ptr<Material> material)
{
	std::shared_ptr<OsgMaterial> osgMaterial = std::dynamic_pointer_cast<OsgMaterial>(material);
	bool result = false;
	if (osgMaterial != nullptr)
	{
		m_materialProxy->setStateSet(osgMaterial->getOsgStateSet());
		result = true;
		m_material = osgMaterial;
	}
	return result;
}

std::shared_ptr<Material> OsgCamera::getMaterial() const
{
	return m_material;
}

void OsgCamera::clearMaterial()
{
	m_materialProxy->setStateSet(new osg::StateSet());
}

void OsgCamera::detachCurrentRenderTarget()
{
	if (m_renderTarget != nullptr)
	{
		if (m_renderTarget->doesUseDepthTarget())
		{
			m_camera->detach(osg::Camera::DEPTH_BUFFER);
		}
		for (int i = 0; i < m_renderTarget->getColorTargetCount(); ++i)
		{
			m_camera->detach(ColorBufferEnums[i]);
		}
	}
	m_renderTarget = nullptr;
}

void OsgCamera::setViewport(int x, int y, int width, int height)
{
	m_camera->setViewport(x, y, width, height);
}

void OsgCamera::getViewport(int* x, int* y, int* width, int* height) const
{
	SURGSIM_ASSERT(x != nullptr && y != nullptr && width != nullptr && height != nullptr)
			<< "Parameter can't be nullptr.";

	auto viewPort = m_camera->getViewport();

	SURGSIM_ASSERT(viewPort != nullptr) << "Trying to access viewport before it has been established.";

	*x = viewPort->x();
	*y = viewPort->y();
	*width = viewPort->width();
	*height = viewPort->height();
}

void OsgCamera::setViewportSize(std::array<double, 2> dimensions)
{
	auto viewPort = m_camera->getViewport();

	if (viewPort != nullptr)
	{
		double aspectRatioChange = (dimensions[0] / viewPort->width()) / (dimensions[1] / viewPort->height());
		m_camera->setViewport(viewPort->x(), viewPort->y(), dimensions[0], dimensions[1]);
		m_camera->getProjectionMatrix() *= osg::Matrix::scale(1.0 / aspectRatioChange, aspectRatioChange,1.0);
		m_projectionMatrix = fromOsg(m_camera->getProjectionMatrix());
	}
	else
	{
		m_camera->setViewport(0, 0, dimensions[0], dimensions[1]);
	}
}

std::array<double, 2> OsgCamera::getViewportSize() const
{
	auto viewPort = m_camera->getViewport();

	SURGSIM_ASSERT(viewPort != nullptr) << "Trying to access viewport before it has been established.";

	std::array<double, 2> dimensions = {viewPort->width(), viewPort->height()};

	return dimensions;
}

void OsgCamera::setMainCamera(bool val)
{
	if (val != m_isMainCamera)
	{
		if (val)
		{
			m_camera->removeChild(m_materialProxy);

			std::array<long, 2> masks = {CullMaskLeft, CullMaskRight}; // NOLINT

			// Insert two nodes into the camera hierarchy, they will update the global uniforms with the correct
			// value. Also attach the material proxy to each of the nodes.
			for (auto& mask : masks)
			{
				auto node = createUniformUpdateNode(mask);
				m_camera->addChild(node);
				node->addChild(m_materialProxy);
			}
		}
		else
		{
			// Remove the update nodes and hook the material proxy node back up
			m_camera->removeChildren(0, m_camera->getNumChildren());
			m_camera->addChild(m_materialProxy);
		}
		m_isMainCamera = val;
	}
}

bool OsgCamera::isMainCamera()
{
	return m_isMainCamera;
}

void OsgCamera::setPerspectiveProjection(double fovy, double aspect, double near, double far)
{
	m_camera->setProjectionMatrixAsPerspective(fovy, aspect, near, far);
	m_projectionMatrix = fromOsg(m_camera->getProjectionMatrix());
}


void OsgCamera::setOrthogonalProjection(double left, double right, double bottom, double top, double near, double far)
{
	m_camera->setProjectionMatrixAsOrtho(left, right, bottom, top, near, far);
	m_projectionMatrix = fromOsg(m_camera->getProjectionMatrix());
}

void OsgCamera::attachRenderTargetTexture(osg::Camera::BufferComponent buffer, std::shared_ptr<Texture> texture)
{
	if (texture == nullptr)
	{
		return;
	}

	std::shared_ptr<OsgTexture> osgTexture = std::dynamic_pointer_cast<OsgTexture>(texture);
	SURGSIM_ASSERT(osgTexture != nullptr)
			<< "RenderTarget used a texture that was not an OsgTexture subclass";

	osg::Texture* actualTexture = osgTexture->getOsgTexture();
	SURGSIM_ASSERT(actualTexture != nullptr) <<
			"Could not find texture";

	m_camera->attach(buffer, actualTexture, 0, 0);
}

void OsgCamera::setRenderOrder(RenderOrder order, int value)
{
	if (order < RENDER_ORDER_COUNT)
	{
		m_camera->setRenderOrder(RenderOrderEnums[order], value);
	}
}

osg::ref_ptr<osg::Camera> OsgCamera::getOsgCamera() const
{
	return m_camera;
}

osg::ref_ptr<osg::Node> OsgCamera::getOsgNode() const
{
	return m_switch;
}

SurgSim::Math::Matrix44d OsgCamera::getInverseViewMatrix() const
{
	return getPose().matrix();
}

void OsgCamera::setAmbientColor(const SurgSim::Math::Vector4d& color)
{
	m_ambientColor = color;
	m_ambientColorUniform->set(color.cast<float>());
}

SurgSim::Math::Vector4d OsgCamera::getAmbientColor()
{
	return m_ambientColor;
}

void OsgCamera::setGenerateTangents(bool value)
{
	SURGSIM_ASSERT(value == false) << "Generate Tangents is not supported on Cameras.";
}

}; // namespace Graphics
}; // namespace SurgSim

