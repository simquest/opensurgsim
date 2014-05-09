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

#include "SurgSim/DataStructures/DataGroupCopier.h"

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/IndexDirectory.h"

using SurgSim::DataStructures::IndexDirectory;

namespace SurgSim
{
namespace DataStructures
{

DataGroupCopier::DataGroupCopier(const DataGroup& source, DataGroup& target) :
	m_source(source),
	m_target(target)
{
	findMap();
}

void DataGroupCopier::copy()
{
	m_target.poses().copy(m_source.poses(), m_map[0]);
	m_target.vectors().copy(m_source.vectors(), m_map[1]);
	m_target.matrices().copy(m_source.matrices(), m_map[2]);
	m_target.scalars().copy(m_source.scalars(), m_map[3]);
	m_target.integers().copy(m_source.integers(), m_map[4]);
	m_target.booleans().copy(m_source.booleans(), m_map[5]);
	m_target.strings().copy(m_source.strings(), m_map[6]);
	m_target.customData().copy(m_source.customData(), m_map[7]);
}

void DataGroupCopier::findMap()
{
	m_map[0] = findMap(m_source.poses().getDirectory(), m_target.poses().getDirectory());
	m_map[1] = findMap(m_source.vectors().getDirectory(), m_target.vectors().getDirectory());
	m_map[2] = findMap(m_source.matrices().getDirectory(), m_target.matrices().getDirectory());
	m_map[3] = findMap(m_source.scalars().getDirectory(), m_target.scalars().getDirectory());
	m_map[4] = findMap(m_source.integers().getDirectory(), m_target.integers().getDirectory());
	m_map[5] = findMap(m_source.booleans().getDirectory(), m_target.booleans().getDirectory());
	m_map[6] = findMap(m_source.strings().getDirectory(), m_target.strings().getDirectory());
	m_map[7] = findMap(m_source.customData().getDirectory(), m_target.customData().getDirectory());
}

NamedDataCopyMap DataGroupCopier::findMap(std::shared_ptr<const IndexDirectory> source,
											   std::shared_ptr<const IndexDirectory> target) const
{
	NamedDataCopyMap map;
	const std::vector<std::string>& sourceNames = source->getAllNames();
	for (auto it = sourceNames.cbegin(); it != sourceNames.cend(); ++it)
	{
		const int targetIndex = target->getIndex(*it);
		if (targetIndex > -1)
		{
			map[source->getIndex(*it)] = targetIndex;
		}
	}
	return map;
}

};  // namespace DataStructures
};  // namespace SurgSim
