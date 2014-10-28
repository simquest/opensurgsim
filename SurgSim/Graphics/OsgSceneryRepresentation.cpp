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

#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgModel.h"

namespace SurgSim
{

namespace Graphics
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Graphics::OsgSceneryRepresentation, OsgSceneryRepresentation);

OsgSceneryRepresentation::OsgSceneryRepresentation(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	SceneryRepresentation(name),
	m_osgNode(nullptr),
	m_fileName()
{
}

bool OsgSceneryRepresentation::doInitialize()
{
	return true;
}

void OsgSceneryRepresentation::loadModel(const std::string& fileName)
{
	auto model = std::make_shared<OsgModel>();
	model->load(fileName);
	setModel(model);
}

void OsgSceneryRepresentation::setModel(std::shared_ptr<SurgSim::Framework::Asset> model)
{
	auto osgModel = std::dynamic_pointer_cast<OsgModel>(model);

	SURGSIM_ASSERT(model == nullptr || osgModel != nullptr) << "OsgSceneryRepresentation expects an OsgModel.";

	if (m_osgNode.valid())
	{
		m_transform->removeChild(m_osgNode);
	}
	if (osgModel != nullptr)
	{
		m_osgNode = osgModel->getOsgNode();
		m_transform->addChild(m_osgNode);
	}

	m_model = osgModel;

}

std::shared_ptr<Model> OsgSceneryRepresentation::getModel() const
{
	return m_model;
}

};	// namespace Graphics
};	// namespace SurgSim