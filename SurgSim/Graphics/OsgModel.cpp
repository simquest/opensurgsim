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

#include "SurgSim/Graphics/OsgModel.h"
#include "SurgSim/Framework/Log.h"

#include <osgDB/ReadFile>


namespace SurgSim
{
namespace Graphics
{

SURGSIM_REGISTER(SurgSim::Framework::Asset, SurgSim::Graphics::OsgModel, OsgModel);

OsgModel::OsgModel()
{
}

OsgModel::~OsgModel()
{

}

osg::ref_ptr<osg::Node> OsgModel::getOsgNode()
{
	return m_root;
}

bool OsgModel::doLoad(const std::string& filePath)
{
	m_root = osgDB::readNodeFile(filePath);
	SURGSIM_ASSERT(m_root.valid()) << "Could not load file " << filePath << std::endl;
	return true;
}


}
}

