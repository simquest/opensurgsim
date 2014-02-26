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

#include "SurgSim/Graphics/OsgRepresentation.h"

#include <algorithm>

#include <boost/thread/locks.hpp>

#include "SurgSim/Framework/Log.h"

#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgRigidTransformConversions.h"
#include "SurgSim/Graphics/OsgUnitBox.h"

#include <osg/Geode>
#include <osg/Switch>
#include <osg/PositionAttitudeTransform>

namespace SurgSim
{
namespace Graphics
{

OsgRepresentation::OsgRepresentation(const std::string& name) :
	Representation(name)
{
	m_switch = new osg::Switch;
	m_switch->setName(name + " Switch");

	m_transform = new osg::PositionAttitudeTransform();
	m_transform->setName(name + " Transform");

	m_switch->addChild(m_transform);

	setInitialPose(SurgSim::Math::RigidTransform3d::Identity());
}

OsgRepresentation::~OsgRepresentation()
{

}

void OsgRepresentation::setVisible(bool visible)
{
	m_switch->setChildValue(m_transform, visible);
}

bool OsgRepresentation::isVisible() const
{
	return m_switch->getChildValue(m_transform);
}

void OsgRepresentation::update(double dt)
{
	doUpdate(dt);
}

void OsgRepresentation::setInitialPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_initialPose = pose;
	setPose(m_initialPose);
}


const SurgSim::Math::RigidTransform3d& OsgRepresentation::getInitialPose() const
{
	return m_initialPose;
}

void OsgRepresentation::setPose(const SurgSim::Math::RigidTransform3d& transform)
{
	// HS-2013-jun-28 This function should probably be protected by a mutes, but I can see
	// the assumption could be that this is only called from on thread.
	// #threadsafety
	m_pose = transform;
	std::pair<osg::Quat, osg::Vec3d> pose = toOsg(m_pose);
	m_transform->setAttitude(pose.first);
	m_transform->setPosition(pose.second);
}

const SurgSim::Math::RigidTransform3d& OsgRepresentation::getPose() const
{
	return m_pose;
}

bool OsgRepresentation::setMaterial(std::shared_ptr<SurgSim::Graphics::Material> material)
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

std::shared_ptr<Material> OsgRepresentation::getMaterial() const
{
	return m_material;
}

void OsgRepresentation::clearMaterial()
{
	m_transform->setStateSet(new osg::StateSet()); // Reset to empty state set
	m_material = nullptr;
}

osg::ref_ptr<osg::Node> OsgRepresentation::getOsgNode() const
{
	return m_switch;
}

void OsgRepresentation::doUpdate(double dt)
{

}

bool OsgRepresentation::addGroupReference(const std::string& name)
{
	bool result = false;
	if (! isAwake())
	{
		auto insertion = m_groups.insert(name);
		result = insertion.second;
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Graphics")) <<
				"Representation::requestGroup() was called while the component was already awake for component " <<
				getName() << " this has no effect and should be avoided.";
	}
	return result;

}

void OsgRepresentation::addGroupReferences(const std::vector<std::string>& groups)
{
	if (! isAwake())
	{
		for (auto it = groups.cbegin(); it != groups.cend(); ++it)
		{
			addGroupReference(*it);
		}
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Graphics")) <<
				"Representation::addGroupReferences() was called while the component " <<
				"was already awake for component " << getName() << " this has no effect and should be avoided.";
	}

}

void OsgRepresentation::setGroupReferences(const std::vector<std::string>& groups)
{
	if (! isAwake())
	{
		m_groups.clear();
		for (auto it = groups.cbegin(); it != groups.cend(); ++it)
		{
			addGroupReference(*it);
		}
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Graphics")) <<
				"Representation::setGroupReferences() was called while the component " <<
				"was already awake for component " << getName() << " this has no effect and should be avoided.";
	}
}

std::vector<std::string> OsgRepresentation::getGroupReferences()
{
	return std::vector<std::string>(std::begin(m_groups), std::end(m_groups));
}

}; // Graphics
}; // SurgSim
