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

#include <SurgSim/Graphics/OsgGroup.h>
#include <SurgSim/Graphics/OsgMatrixConversions.h>
#include <SurgSim/Graphics/OsgQuaternionConversions.h>
#include <SurgSim/Graphics/OsgVectorConversions.h>

using SurgSim::Graphics::OsgActor;
using SurgSim::Graphics::OsgCamera;
using SurgSim::Graphics::OsgGroup;
using SurgSim::Graphics::fromOsg;
using SurgSim::Graphics::toOsg;
using SurgSim::Math::makeRigidTransform;

OsgCamera::OsgCamera(const std::string& name) : SurgSim::Graphics::Actor(name), SurgSim::Graphics::Camera(name),
	OsgActor(name, new osg::Switch()),
	m_camera(new osg::Camera())
{
	m_switch = static_cast<osg::Switch*>(getOsgNode().get());
	m_switch->setName(name + " Switch");
	m_camera->setName(name + " Camera");

	m_switch->addChild(m_camera);

	/// Update pose to inverse of view matrix
	osg::Matrixd inverseViewMatrix = osg::Matrixd::inverse(m_camera->getViewMatrix());
	m_pose = makeRigidTransform(fromOsg<double>(inverseViewMatrix.getRotate()), fromOsg(inverseViewMatrix.getTrans()));

	/// Update storage of view and projection matrices
	m_viewMatrix = fromOsg(m_camera->getViewMatrix());
	m_projectionMatrix = fromOsg(m_camera->getProjectionMatrix());
}

bool OsgCamera::setGroup(std::shared_ptr<SurgSim::Graphics::Group> group)
{
	std::shared_ptr<OsgGroup> osgGroup = std::dynamic_pointer_cast<OsgGroup>(group);
	if (osgGroup && SurgSim::Graphics::Camera::setGroup(group))
	{
		m_camera->setChild(0, osgGroup->getOsgGroup());
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