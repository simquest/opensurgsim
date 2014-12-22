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

DataGroupCopier::DataGroupCopier(const DataGroup& source, DataGroup* target)
{
	SURGSIM_ASSERT(target != nullptr) << "Target is a nullptr";
	m_map[0] = findMap(source.poses().getDirectory(), target->poses().getDirectory());
	m_map[1] = findMap(source.vectors().getDirectory(), target->vectors().getDirectory());
	m_map[2] = findMap(source.matrices().getDirectory(), target->matrices().getDirectory());
	m_map[3] = findMap(source.scalars().getDirectory(), target->scalars().getDirectory());
	m_map[4] = findMap(source.integers().getDirectory(), target->integers().getDirectory());
	m_map[5] = findMap(source.booleans().getDirectory(), target->booleans().getDirectory());
	m_map[6] = findMap(source.strings().getDirectory(), target->strings().getDirectory());
	m_map[7] = findMap(source.images().getDirectory(), target->images().getDirectory());
	m_map[8] = findMap(source.customData().getDirectory(), target->customData().getDirectory());
}

void DataGroupCopier::copy(const DataGroup& source, DataGroup* target)
{
	SURGSIM_ASSERT(target != nullptr) << "Target is a nullptr";
	target->poses().copy(source.poses(), m_map[0]);
	target->vectors().copy(source.vectors(), m_map[1]);
	target->matrices().copy(source.matrices(), m_map[2]);
	target->scalars().copy(source.scalars(), m_map[3]);
	target->integers().copy(source.integers(), m_map[4]);
	target->booleans().copy(source.booleans(), m_map[5]);
	target->strings().copy(source.strings(), m_map[6]);
	target->images().copy(source.images(), m_map[7]);
	target->customData().copy(source.customData(), m_map[8]);
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
