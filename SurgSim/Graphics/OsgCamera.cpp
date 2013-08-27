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

#include <SurgSim/Graphics/OsgCamera.h>

#include <SurgSim/Graphics/Material.h>
#include <SurgSim/Graphics/Manager.h>
#include <SurgSim/Graphics/OsgGroup.h>
#include <SurgSim/Graphics/OsgMatrixConversions.h>
#include <SurgSim/Graphics/OsgQuaternionConversions.h>
#include <SurgSim/Graphics/OsgVectorConversions.h>
#include <SurgSim/Graphics/OsgRenderTarget.h>


using SurgSim::Math::makeRigidTransform;

namespace {
	const osg::Camera::BufferComponent ColorBufferEnums[16] = {
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
};


namespace SurgSim
{
namespace Graphics
{

OsgCamera::OsgCamera(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	Camera(name),
	m_camera(new osg::Camera())
{
	m_switch->removeChildren(0, m_switch->getNumChildren());
	m_camera->setName(name + " Camera");

	m_switch->addChild(m_camera);

	/// Update pose to inverse of view matrix
	osg::Matrixd inverseViewMatrix = osg::Matrixd::inverse(m_camera->getViewMatrix());
	m_pose = makeRigidTransform(fromOsg<double>(inverseViewMatrix.getRotate()), fromOsg(inverseViewMatrix.getTrans()));

	m_camera->setViewMatrixAsLookAt(osg::Vec3d(0.0, 0.0, 0.0), osg::Vec3d(0.0, 0.0, -1.0), osg::Vec3d(0.0, 1.0, 0.0));
	m_camera->setProjectionMatrixAsPerspective(45.0, 1.0, 0.01, 10.0);

	/// Update storage of view and projection matrices
	m_viewMatrix = fromOsg(m_camera->getViewMatrix());
	m_projectionMatrix = fromOsg(m_camera->getProjectionMatrix());
}

bool OsgCamera::setGroup(std::shared_ptr<SurgSim::Graphics::Group> group)
{
	std::shared_ptr<OsgGroup> osgGroup = std::dynamic_pointer_cast<OsgGroup>(group);
	if (osgGroup && SurgSim::Graphics::Camera::setGroup(group))
	{

		m_camera->removeChildren(0, m_camera->getNumChildren());  /// Remove any previous group
		m_camera->addChild(osgGroup->getOsgGroup());
		return true;
	}
	else
	{
		return false;
	}
}

void OsgCamera::setVisible(bool visible)
{
	m_switch->setChildValue(m_camera, visible);
}

bool OsgCamera::isVisible() const
{
	return m_switch->getChildValue(m_camera);
}

void OsgCamera::setPose(const SurgSim::Math::RigidTransform3d& transform)
{
	setViewMatrix(transform.matrix().inverse());
}

const SurgSim::Math::RigidTransform3d& OsgCamera::getPose() const
{
	return m_pose;
}

void OsgCamera::setViewMatrix(const SurgSim::Math::Matrix44d& matrix)
{
	m_viewMatrix = matrix;

	/// Set the pose to the inverse of the view matrix
	osg::Matrixd osgViewMatrix = toOsg(matrix);
	osg::Matrixd osgInverseViewMatrix = osg::Matrixd::inverse(osgViewMatrix);
	m_pose = makeRigidTransform(fromOsg<double>(osgInverseViewMatrix.getRotate()),
								fromOsg(osgInverseViewMatrix.getTrans()));

	m_camera->setViewMatrix(osgViewMatrix);
}

const SurgSim::Math::Matrix44d& OsgCamera::getViewMatrix() const
{
	return m_viewMatrix;
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

void OsgCamera::update(double dt)
{
	/// Update pose to inverse of view matrix
	osg::Matrixd inverseViewMatrix = osg::Matrixd::inverse(m_camera->getViewMatrix());
	m_pose = makeRigidTransform(fromOsg<double>(inverseViewMatrix.getRotate()), fromOsg(inverseViewMatrix.getTrans()));

	/// Update storage of view and projection matrices
	m_viewMatrix = fromOsg(m_camera->getViewMatrix());
	m_projectionMatrix = fromOsg(m_camera->getProjectionMatrix());
}

bool OsgCamera::setMaterial(std::shared_ptr<Material> material)
{
	SURGSIM_FAILURE() << "You cannot assign a material to a camera node";
	return false;
}

std::shared_ptr<Material> OsgCamera::getMaterial() const
{
	SURGSIM_FAILURE() << "A camera node does not have a material";
	return nullptr;
}

void OsgCamera::clearMaterial()
{
	SURGSIM_FAILURE() << "A camera node does not have a material";
}

bool OsgCamera::setColorRenderTexture(std::shared_ptr<Texture> texture)
{
	std::shared_ptr<OsgTexture> osgTexture = std::dynamic_pointer_cast<OsgTexture>(texture);
	osg::Texture* actualTexture;

	actualTexture = osgTexture->getOsgTexture();

	bool result = false;
	if (actualTexture != nullptr )
	{
		m_camera->attach(osg::Camera::COLOR_BUFFER, actualTexture, 0, 0);
		m_camera->setRenderOrder(osg::Camera::PRE_RENDER);
		m_textureMap[osg::Camera::COLOR_BUFFER] = osgTexture;
		m_camera->setClearColor(osg::Vec4f(0.0, 0.0, 0.0, 1.0));
		m_camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT, osg::Camera::PIXEL_BUFFER);
		m_camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);

		int width = actualTexture->getTextureWidth();
		int height = actualTexture->getTextureHeight();
		m_camera->setViewport(0,0,width,height);

		result = m_camera->isRenderToTextureCamera();
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) << __FUNCTION__ <<
			"Texture passed as ColorRenderTexture is null.";
	}

	return result;
}

std::shared_ptr<Texture> OsgCamera::getColorRenderTexture() const
{
	return m_textureMap.at(osg::Camera::COLOR_BUFFER);
}

void OsgCamera::setRenderTarget(std::shared_ptr<RenderTarget> renderTarget)
{
	if (m_renderTarget != nullptr)
	{
		detachCurrentRenderTarget();
	}

	int width, height;
	renderTarget->getSize(&width, &height);
	m_camera->setViewport(0,0,width,height);

	attachRenderTargetTexture(osg::Camera::DEPTH_BUFFER, renderTarget->getDepthTarget());

	// OSG has 16 COLOR_BUFFER objects
	for (int i = 0; i < 16; ++i)
	{
		attachRenderTargetTexture(ColorBufferEnums[i], renderTarget->getColorTarget(i));
	}

	m_camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT, osg::Camera::PIXEL_BUFFER);
	m_camera->setRenderOrder(osg::Camera::PRE_RENDER);
	m_camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	m_camera->setClearColor(osg::Vec4f(0.0, 0.0, 0.0, 1.0));
	m_renderTarget = renderTarget;
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

void OsgCamera::attachRenderTargetTexture(osg::Camera::BufferComponent buffer, std::shared_ptr<Texture> texture)
{
	if (texture == nullptr)
	{
		return;
	}

	std::shared_ptr<OsgTexture> osgTexture = std::dynamic_pointer_cast<OsgTexture>(texture);
	SURGSIM_ASSERT(osgTexture != nullptr) <<
		"RenderTarget used a texture that was not an OsgTexture subclass";

	osg::Texture* actualTexture = osgTexture->getOsgTexture();
	SURGSIM_ASSERT(actualTexture != nullptr) <<
		"Could not find texture";

	m_camera->attach(buffer, actualTexture, 0, 0);
}

}; // namespace Graphics
}; // namespace SurgSim

