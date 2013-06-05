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

#include <SurgSim/Graphics/OsgSphereActor.h>

#include <SurgSim/Graphics/OsgRigidTransformConversions.h>
#include <SurgSim/Graphics/OsgUnitSphere.h>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>

using SurgSim::Graphics::OsgSphereActor;
using SurgSim::Graphics::OsgUnitSphere;

OsgSphereActor::OsgSphereActor(const std::string& name) : Actor(name), SphereActor(name),
	OsgActor(name, new osg::Switch()),
	m_sharedUnitSphere(getSharedUnitSphere())
{
	m_switch = static_cast<osg::Switch*>(getOsgNode().get());
	m_switch->setName(name + " Switch");

	m_transform = new osg::PositionAttitudeTransform();
	m_switch->setName(name + " Transform");
	m_transform->addChild(m_sharedUnitSphere->getNode());

	m_switch->addChild(m_transform);

	std::pair<osg::Quat, osg::Vec3d> pose = std::make_pair(m_transform->getAttitude(), m_transform->getPosition());
	m_pose = fromOsg(pose);
}

void OsgSphereActor::setVisible(bool visible)
{
	m_switch->setChildValue(m_transform, visible);
}

bool OsgSphereActor::isVisible() const
{
	return m_switch->getChildValue(m_transform);
}

void OsgSphereActor::setRadius(double radius)
{
	m_transform->setScale(osg::Vec3d(radius, radius, radius));
}
double OsgSphereActor::getRadius() const
{
	SURGSIM_ASSERT(m_transform->getScale().x() == m_transform->getScale().y() &&
		m_transform->getScale().x() == m_transform->getScale().z()) << "Sphere should be scaled equally in all directions!";
	return m_transform->getScale().x();
}

void OsgSphereActor::setPose(const SurgSim::Math::RigidTransform3d& transform)
{
	m_pose = transform;
	std::pair<osg::Quat, osg::Vec3d> pose = toOsg(m_pose);
	m_transform->setAttitude(pose.first);
	m_transform->setPosition(pose.second);
}

const SurgSim::Math::RigidTransform3d& OsgSphereActor::getPose() const
{
	return m_pose;
}

void OsgSphereActor::update(double dt)
{
}

std::shared_ptr<OsgUnitSphere> OsgSphereActor::getSharedUnitSphere()
{
	static SurgSim::Framework::SharedInstance<OsgUnitSphere> shared;
	return shared.get();
}
