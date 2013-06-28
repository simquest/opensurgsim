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

#include <SurgSim/Graphics/OsgBoxRepresentation.h>

#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgRigidTransformConversions.h>
#include <SurgSim/Graphics/OsgUnitBox.h>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>

using SurgSim::Graphics::OsgBoxRepresentation;
using SurgSim::Graphics::OsgUnitBox;

OsgBoxRepresentation::OsgBoxRepresentation(const std::string& name) : Representation(name), BoxRepresentation(name),
	OsgRepresentation(name, new osg::Switch()),
	m_scale(1.0, 1.0, 1.0),
	m_sharedUnitBox(getSharedUnitBox())
{
	m_switch = static_cast<osg::Switch*>(getOsgNode().get());
	m_switch->setName(name + " Switch");

	m_transform = new osg::PositionAttitudeTransform();
	m_switch->setName(name + " Transform");
	m_transform->addChild(m_sharedUnitBox->getNode());

	m_switch->addChild(m_transform);

	std::pair<osg::Quat, osg::Vec3d> pose = std::make_pair(m_transform->getAttitude(), m_transform->getPosition());
	m_pose = fromOsg(pose);
}

void OsgBoxRepresentation::setVisible(bool visible)
{
	m_switch->setChildValue(m_transform, visible);
}

bool OsgBoxRepresentation::isVisible() const
{
	return m_switch->getChildValue(m_transform);
}

bool OsgBoxRepresentation::setMaterial(std::shared_ptr<SurgSim::Graphics::Material> material)
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

void OsgBoxRepresentation::clearMaterial()
{
	m_transform->setStateSet(new osg::StateSet()); // Reset to empty state set
	Representation::setMaterial(nullptr);
}

void OsgBoxRepresentation::setSizeX(double sizeX)
{
	m_scale.x() = sizeX;
	m_transform->setScale(m_scale);
}
double OsgBoxRepresentation::getSizeX() const
{
	return m_scale.x();
}

void OsgBoxRepresentation::setSizeY(double sizeY)
{
	m_scale.y() = sizeY;
	m_transform->setScale(m_scale);
}
double OsgBoxRepresentation::getSizeY() const
{
	return m_scale.y();
}

void OsgBoxRepresentation::setSizeZ(double sizeZ)
{
	m_scale.z() = sizeZ;
	m_transform->setScale(m_scale);
}
double OsgBoxRepresentation::getSizeZ() const
{
	return m_scale.z();
}

void OsgBoxRepresentation::setSize(double sizeX, double sizeY, double sizeZ)
{
	m_scale.x() = sizeX;
	m_scale.y() = sizeY;
	m_scale.z() = sizeZ;
	m_transform->setScale(m_scale);
}
void OsgBoxRepresentation::getSize(double& sizeX, double& sizeY, double& sizeZ)
{
	sizeX =	m_scale.x();
	sizeY = m_scale.y();
	sizeZ = m_scale.z();
}

void OsgBoxRepresentation::setSize(SurgSim::Math::Vector3d size)
{
	m_scale.set(size.x(), size.y(), size.z());
	m_transform->setScale(m_scale);
}
SurgSim::Math::Vector3d OsgBoxRepresentation::getSize() const
{
	return SurgSim::Math::Vector3d(m_scale._v);
}


void OsgBoxRepresentation::setPose(const SurgSim::Math::RigidTransform3d& transform)
{
	m_pose = transform;
	std::pair<osg::Quat, osg::Vec3d> pose = toOsg(m_pose);
	m_transform->setAttitude(pose.first);
	m_transform->setPosition(pose.second);
}

const SurgSim::Math::RigidTransform3d& OsgBoxRepresentation::getPose() const
{
	return m_pose;
}

void OsgBoxRepresentation::update(double dt)
{
}

std::shared_ptr<OsgUnitBox> OsgBoxRepresentation::getSharedUnitBox()
{
	static SurgSim::Framework::SharedInstance<OsgUnitBox> shared;
	return shared.get();
}
