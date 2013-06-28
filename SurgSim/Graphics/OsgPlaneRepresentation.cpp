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

#include <SurgSim/Graphics/OsgPlaneRepresentation.h>

#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgPlane.h>
#include <SurgSim/Graphics/OsgRigidTransformConversions.h>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>

using SurgSim::Graphics::OsgPlaneRepresentation;
using SurgSim::Graphics::OsgPlane;

OsgPlaneRepresentation::OsgPlaneRepresentation(const std::string& name) :
	Representation(name),
	PlaneRepresentation(name),
	OsgRepresentation(name, new osg::Switch()),
	m_sharedPlane(getSharedPlane())
{
	m_switch = static_cast<osg::Switch*>(getOsgNode().get());
	m_switch->setName(name + " Switch");

	m_transform = new osg::PositionAttitudeTransform();
	m_switch->setName(name + " Transform");
	m_transform->addChild(m_sharedPlane->getNode());

	m_switch->addChild(m_transform);

	std::pair<osg::Quat, osg::Vec3d> pose = std::make_pair(m_transform->getAttitude(), m_transform->getPosition());
	m_pose = fromOsg(pose);
}

void OsgPlaneRepresentation::setVisible(bool visible)
{
	m_switch->setChildValue(m_transform, visible);
}

bool OsgPlaneRepresentation::isVisible() const
{
	return m_switch->getChildValue(m_transform);
}

bool OsgPlaneRepresentation::setMaterial(std::shared_ptr<SurgSim::Graphics::Material> material)
{
	bool didSucceed = false;

	std::shared_ptr<OsgMaterial> osgMaterial = std::dynamic_pointer_cast<OsgMaterial>(material);
	if (osgMaterial && Representation::setMaterial(material))
	{
		m_transform->setStateSet(osgMaterial->getOsgStateSet());
		didSucceed = true;
	}
	return didSucceed;
}

void OsgPlaneRepresentation::clearMaterial()
{
	m_transform->setStateSet(new osg::StateSet()); // Reset to empty state set
	Representation::setMaterial(nullptr);
}

void OsgPlaneRepresentation::setPose(const SurgSim::Math::RigidTransform3d& transform)
{
	m_pose = transform;
	std::pair<osg::Quat, osg::Vec3d> pose = toOsg(m_pose);
	m_transform->setAttitude(pose.first);
	m_transform->setPosition(pose.second);
}

const SurgSim::Math::RigidTransform3d& OsgPlaneRepresentation::getPose() const
{
	return m_pose;
}

void OsgPlaneRepresentation::update(double dt)
{
}

std::shared_ptr<OsgPlane> OsgPlaneRepresentation::getSharedPlane()
{
	static SurgSim::Framework::SharedInstance<OsgPlane> shared;
	return shared.get();
}
