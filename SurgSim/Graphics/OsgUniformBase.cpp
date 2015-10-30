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

#include "SurgSim/Graphics/OsgUniformBase.h"
#include "SurgSim/Framework/Log.h"

using SurgSim::Graphics::OsgUniformBase;

OsgUniformBase::OsgUniformBase(const std::string& name) : UniformBase(),
	m_uniform(new osg::Uniform())
{
	m_uniform->setName(name);
}

void OsgUniformBase::addToStateSet(osg::StateSet* stateSet)
{
	SURGSIM_LOG_DEBUG(Framework::Logger::getLogger("Graphics/OsgUniformBase")) << "Base Add To Texture StateSet called";
	stateSet->addUniform(m_uniform);
}

void OsgUniformBase::removeFromStateSet(osg::StateSet* stateSet)
{
	stateSet->removeUniform(m_uniform);
}
