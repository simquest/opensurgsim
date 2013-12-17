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

#include "SurgSim/Graphics/OsgGroup.h"

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Graphics/OsgRepresentation.h"

using SurgSim::Graphics::OsgRepresentation;
using SurgSim::Graphics::OsgGroup;

OsgGroup::OsgGroup(const std::string& name) : SurgSim::Graphics::Group(name),
	m_isVisible(true),
	m_switch(new osg::Switch())
{
	m_switch->setName(name + " Switch");
};

void OsgGroup::setVisible(bool visible)
{
	m_isVisible = visible;

	if (m_isVisible)
	{
		m_switch->setAllChildrenOn();
	}
	else
	{
		m_switch->setAllChildrenOff();
	}
}

bool OsgGroup::isVisible() const
{
	return m_isVisible;
}

bool OsgGroup::add(std::shared_ptr<SurgSim::Graphics::Representation> representation)
{
	std::shared_ptr<OsgRepresentation> osgRepresentation = std::dynamic_pointer_cast<OsgRepresentation>(representation);

	if (osgRepresentation && Group::add(osgRepresentation))
	{
		m_switch->addChild(osgRepresentation->getOsgNode());
		m_switch->setChildValue(osgRepresentation->getOsgNode(), m_isVisible);
		return true;
	}
	else
	{
		return false;
	}
}

bool OsgGroup::append(std::shared_ptr<SurgSim::Graphics::Group> group)
{
	std::shared_ptr<OsgGroup> osgGroup = std::dynamic_pointer_cast<OsgGroup>(group);

	if (osgGroup && Group::append(osgGroup))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool OsgGroup::remove(std::shared_ptr<SurgSim::Graphics::Representation> representation)
{
	std::shared_ptr<OsgRepresentation> osgRepresentation = std::dynamic_pointer_cast<OsgRepresentation>(representation);

	if (osgRepresentation && Group::remove(osgRepresentation))
	{
		m_switch->removeChild(osgRepresentation->getOsgNode());
		return true;
	}
	else
	{
		return false;
	}
}

void OsgGroup::clear()
{
	while (!getMembers().empty())
	{
		std::shared_ptr<Representation> representation = getMembers().front();
		SURGSIM_ASSERT(remove(representation)) << "Removal of representation " << representation->getName() <<
			" failed while attempting to clear group " << getName() << "!";
	}
}

osg::ref_ptr<osg::Group> OsgGroup::getOsgGroup() const
{
	return m_switch;
}