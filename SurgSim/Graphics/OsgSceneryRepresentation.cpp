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

#include <SurgSim/Framework/ApplicationData.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Graphics/OsgSceneryObjectRepresentation.h>

#include <osgDB/ReadFile>

using SurgSim::Graphics::OsgSceneryObjectRepresentation;
using SurgSim::Graphics::OsgRepresentation;

OsgSceneryObjectRepresentation::OsgSceneryObjectRepresentation(const std::string& name, const std::string& filePath) :
	Representation(name),
	OsgRepresentation(name),
	m_object(nullptr),
	m_filePath(filePath)
{
}

osg::ref_ptr<osg::Object> OsgSceneryObjectRepresentation::getOsgSceneryObjectRepresentation() const
{
	return m_sceneryObjectRepresentation;
}

bool OsgSceneryObjectRepresentation::doInitialize()
{
	std::shared_ptr<const SurgSim::Framework::ApplicationData> applicationData(getRuntime()->getApplicationData());

	std::string objectPath = applicationData->findFile(m_filePath);
	if (objectPath.empty())
	{
		return false;
	}
	
	m_sceneryObjectRepresentation = osgDB::readNodeFile(objectPath);
	//If the object is loaded correctly, return true; Otherwise, false.
	if (m_sceneryObjectRepresentation.valid())
	{
		return true;
	}
	else
	{
		return false;
	}
}
