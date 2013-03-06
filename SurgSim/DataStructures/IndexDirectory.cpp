// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

#include "SurgSim/DataStructures/IndexDirectory.h"

namespace SurgSim
{
namespace DataStructures
{


IndexDirectory::IndexDirectory()
{
}

IndexDirectory::IndexDirectory(const std::vector<std::string>& names)
{
	for (auto it = names.cbegin();  it != names.cend();  ++it)
	{
		addEntry(*it);
	}
}

const std::vector<std::string>& IndexDirectory::getAllNames() const
{
	return m_names;
}

IndexDirectory::IndexDirectory(const IndexDirectory& directory) :
	m_names(directory.m_names), m_indices(directory.m_indices)
{
}

IndexDirectory& IndexDirectory::operator =(const IndexDirectory& directory)
{
	m_names = directory.m_names;
	m_indices = directory.m_indices;
	return *this;
}

int IndexDirectory::addEntry(const std::string& name)
{
	if ((name.length() == 0) || hasEntry(name))
	{
		return -1;
	}
	int index = static_cast<int>(m_names.size());
	m_names.push_back(name);
	m_indices[name] = index;
	return index;
}


};  // namespace Input
};  // namespace SurgSim
