// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//OsgSceneryObject
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

#include <SurgSim/Framework/ApplicationData.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Graphics/OsgSceneryObject.h>

#include <osgDB/ReadFile>
#include <osg/Switch>

using SurgSim::Graphics::OsgSceneryObject;
using SurgSim::Graphics::OsgRepresentation;

OsgSceneryObject::OsgSceneryObject(const std::string& name, const std::string& filePath) :
	Representation(name),
	OsgRepresentation(name),
	m_sceneryObject(nullptr),
	m_filePath(filePath)
{
}

osg::ref_ptr<osg::Node> OsgSceneryObject::getOsgSceneryObject() const
{
	return m_sceneryObject;
}

bool OsgSceneryObject::doInitialize()
{
	std::shared_ptr<const SurgSim::Framework::ApplicationData> applicationData = getRuntime()->getApplicationData();

	std::string objectPath = applicationData->findFile(m_filePath);
	if (objectPath.empty())
	{
		return false;
	}

	m_sceneryObject = osgDB::readNodeFile(objectPath);
	//If the object is loaded correctly, add it to internal m_switch and return true;
	//Otherwise, false.
	if (m_sceneryObject.valid())
	{
		m_switch->addChild(m_sceneryObject);
		return true;
	}
	else
	{
		return false;
	}
}
