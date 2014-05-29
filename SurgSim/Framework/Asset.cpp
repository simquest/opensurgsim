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

namespace SurgSim
{
namespace Framework
{

Asset::Asset() : m_hasBeenInitialized(false), m_isInitialized(false), m_fileName()
{
}

Asset::~Asset()
{
}

void Asset::setFileName(const std::string& fileName)
{
	m_fileName = fileName;
}

std::string Asset::getFileName() const
{
	return m_fileName;
}

bool Asset::initialize(const std::shared_ptr<ApplicationData>& data)
{
	SURGSIM_ASSERT(!m_hasBeenInitialized) << "Initialization has been called before";
	m_hasBeenInitialized = true;

	std::string path = data->findFile(m_fileName);
	SURGSIM_ASSERT(!path.empty()) << "Can not locate file " << m_fileName;

	m_isInitialized = doInitialize(path);

	return m_isInitialized;
}

bool Asset::isInitialized() const
{
	return m_isInitialized;
}

}; // Framework
}; // SurgSim