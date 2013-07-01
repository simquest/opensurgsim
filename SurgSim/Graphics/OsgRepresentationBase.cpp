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

#include <SurgSim/Graphics/OsgRepresentationBase.h>

#include <boost/thread/locks.hpp>

#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgRigidTransformConversions.h>
#include <SurgSim/Graphics/OsgUnitBox.h>
#include <SurgSim/Graphics/OsgMaterial.h>

#include <osg/Geode>
#include <osg/Switch>
#include <osg/PositionAttitudeTransform>

namespace SurgSim
{
namespace Graphics
{

OsgRepresentationBase::OsgRepresentationBase(const std::string& name)
{
	m_switch = new osg::Switch;
	m_switch->setName(name + " Switch");

	m_transform = new osg::PositionAttitudeTransform();
	m_switch->setName(name + " Transform");

	m_switch->addChild(m_transform);
	
	setInitialPose(SurgSim::Math::RigidTransform3d::Identity());
}

OsgRepresentationBase::~OsgRepresentationBase()
{

}

void OsgRepresentationBase::setVisible(bool visible)
{
	m_switch->setChildValue(m_transform, visible);
}

bool OsgRepresentationBase::isVisible() const
{
	return m_switch->getChildValue(m_transform);
}

void OsgRepresentationBase::update(double dt)
{
	doUpdate(dt);
}

void OsgRepresentationBase::setInitialPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_initialPose = pose;
	setPose(m_initialPose);
}


const SurgSim::Math::RigidTransform3d& OsgRepresentationBase::getInitialPose() const
{
	return m_initialPose;
}

void OsgRepresentationBase::setPose(const SurgSim::Math::RigidTransform3d& transform)
{
	// HS-2013-jun-28 This function should probably be protected by a mutes, but I can see
	// the assumption could be that this is only called from on thread ... right ?
	// #threadsafety
	m_pose = transform;
	std::pair<osg::Quat, osg::Vec3d> pose = toOsg(m_pose);
	m_transform->setAttitude(pose.first);
	m_transform->setPosition(pose.second);
}

const SurgSim::Math::RigidTransform3d& OsgRepresentationBase::getPose() const
{
	return m_pose;
}

bool OsgRepresentationBase::setMaterial(std::shared_ptr<SurgSim::Graphics::Material> material)
{
	bool didSucceed = false;

	std::shared_ptr<OsgMaterial> osgMaterial = std::dynamic_pointer_cast<OsgMaterial>(material);
	if (osgMaterial != nullptr)
	{
		m_transform->setStateSet(osgMaterial->getOsgStateSet());
		didSucceed = true;
		m_material = osgMaterial;
	}
	return didSucceed;
}

std::shared_ptr<Material> OsgRepresentationBase::getMaterial() const
{
	return m_material;
}

void OsgRepresentationBase::clearMaterial()
{
	m_transform->setStateSet(new osg::StateSet()); // Reset to empty state set
	m_material = nullptr;
}

void doUpdate(double dt)
{

}

}; // Graphics
}; // SurgSim
