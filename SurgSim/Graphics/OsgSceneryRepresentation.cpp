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

#include "SurgSim/Graphics/OsgSceneryRepresentation.h"

#include <osg/PositionAttitudeTransform>
#include <osg/Switch>
#include <osgDB/ReadFile>

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Runtime.h"

namespace
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Graphics::OsgSceneryRepresentation);
}

namespace SurgSim
{

namespace Graphics
{

OsgSceneryRepresentation::OsgSceneryRepresentation(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	SceneryRepresentation(name),
	m_sceneryRepresentation(nullptr),
	m_fileName()
{
}

bool OsgSceneryRepresentation::doInitialize()
{
	// HS-2013-dec-17 This should probably use a return value of false rather than assertions
	SURGSIM_ASSERT(m_fileName != "") << "Filename can't be empty.";
	std::shared_ptr<const SurgSim::Framework::ApplicationData> applicationData = getRuntime()->getApplicationData();

	std::string objectPath = applicationData->findFile(m_fileName);
	SURGSIM_ASSERT(!objectPath.empty()) << "Could not find file " << m_fileName << std::endl;

	m_sceneryRepresentation = osgDB::readNodeFile(objectPath);
	SURGSIM_ASSERT(m_sceneryRepresentation.valid()) << "Could not load file " << objectPath << std::endl;

	m_transform->addChild(m_sceneryRepresentation);
	return true;
}

std::string OsgSceneryRepresentation::getFileName() const
{
	return m_fileName;
}

void OsgSceneryRepresentation::setFileName(const std::string& fileName)
{
	SURGSIM_ASSERT(!isInitialized()) << "Can't set the filename after the object has been initialized.";
	m_fileName = fileName;
}

};	// namespace Graphics
};	// namespace SurgSim