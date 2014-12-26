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

#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Framework/SharedInstance.h"

#include <osg/PositionAttitudeTransform>


namespace SurgSim
{
namespace Graphics
{

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Graphics::OsgAxesRepresentation, OsgAxesRepresentation);

OsgAxesRepresentation::OsgAxesRepresentation(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	AxesRepresentation(name),
	m_sharedUnitAxes(getShareUnitAxes()),
	m_size(1.0)
{
	m_transform->addChild(m_sharedUnitAxes->getNode());
}

OsgAxesRepresentation::~OsgAxesRepresentation()
{

}

std::shared_ptr<OsgUnitAxes> OsgAxesRepresentation::getShareUnitAxes()
{
	static SurgSim::Framework::SharedInstance<OsgUnitAxes> shared;
	return shared.get();
}

void OsgAxesRepresentation::setSize(double val)
{
	m_size = val;
	m_transform->setScale(osg::Vec3d(val, val, val));
}

double OsgAxesRepresentation::getSize()
{
	return m_size;
}

}; // Graphics
}; // SurgSim
