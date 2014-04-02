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

#ifndef SURGSIM_DATASTRUCTURES_AABBTREEDATA_H
#define SURGSIM_DATASTRUCTURES_AABBTREEDATA_H

#include "SurgSim/DataStructures/TreeData.h"

#include "SurgSim/Math/Aabb.h"

#include <utility>
#include <list>
#include <memory>

namespace SurgSim
{
namespace DataStructures
{

class AabbTreeData : public TreeData
{
public:
	AabbTreeData();

	AabbTreeData(const AabbTreeData& data);

	~AabbTreeData();

	typedef std::pair<SurgSim::Math::Aabbd, size_t> Item;

	void add(const SurgSim::Math::Aabbd aabb, size_t id);

	const SurgSim::Math::Aabbd& getAabb() const;

	bool isEmpty() const;

	size_t getSize() const;

	std::shared_ptr<AabbTreeData> split();

	bool hasIntersections(const SurgSim::Math::Aabbd& aabb) const;

	void getIntersections(const SurgSim::Math::Aabbd& aabb, std::list<size_t>* ids) const;

protected:

private:
	void recalculateAabb();

	virtual bool isEqual(const TreeData* data) const;

	SurgSim::Math::Aabbd m_aabb;
	std::list<Item> m_data;
};

}
}

#endif
