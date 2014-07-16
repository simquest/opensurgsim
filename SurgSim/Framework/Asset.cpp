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

#include "SurgSim/Framework/Asset.h"

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Runtime.h"

namespace SurgSim
{
namespace Framework
{

Asset::Asset() : m_fileName()
{
}

Asset::~Asset()
{
}

bool Asset::load(const std::string& fileName)
{
	bool result = false;
	m_fileName = fileName;

	auto data = SurgSim::Framework::Runtime::getApplicationData().get();
	std::string path = data->findFile(m_fileName);

	if (path.empty())
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) << __FUNCTION__ <<
			"Can not locate file " << m_fileName;
	}
	else
	{
		result = doLoad(path);
	}

	return result;
}

void Asset::setFileName(const std::string& fileName)
{
	m_fileName = fileName;
	load(m_fileName);
}

std::string Asset::getFileName() const
{
	return m_fileName;
}

}; // Framework
}; // SurgSim