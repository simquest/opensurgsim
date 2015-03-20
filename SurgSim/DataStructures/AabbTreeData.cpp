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

#include "SurgSim/DataStructures/AabbTreeData.h"
#include "SurgSim/Framework/Assert.h"

#include <algorithm>

namespace SurgSim
{
namespace DataStructures
{

AabbTreeData::AabbTreeData()
{

}

AabbTreeData::AabbTreeData(const std::list<Item>& data)
{
	m_data = data;
	recalculateAabb();
}

AabbTreeData::AabbTreeData(std::list<Item>&& data)
{
	std::swap(m_data, data);
	recalculateAabb();
}

AabbTreeData::~AabbTreeData()
{

}

bool AabbTreeData::isEqual(const TreeData* data) const
{
	// The type safety of this is guaranteed by the == operator in TreeData
	const AabbTreeData* treeData = static_cast<const AabbTreeData*>(data);
	bool result = false;
	if (getSize() == treeData->getSize() && getAabb().isApprox(treeData->getAabb()))
	{
		result = true;
		auto it = m_data.cbegin();
		auto endIt = m_data.cend();
		auto otherEnd = treeData->m_data.cend();
		auto functor = [it](const Item & other)
		{
			return ((*it).second == other.second && (*it).first.isApprox(other.first));
		};
		for (; it != endIt; ++it)
		{
			if (std::find_if(treeData->m_data.cbegin(), treeData->m_data.cend(), functor) == otherEnd)
			{
				result = false;
				break;
			}
		}

	}
	return result;
}

void AabbTreeData::add(const SurgSim::Math::Aabbd aabb, size_t id)
{
	m_aabb.extend(aabb);
	m_data.emplace_back(aabb, id);
}

const SurgSim::Math::Aabbd& AabbTreeData::getAabb() const
{
	return m_aabb;
}


bool AabbTreeData::isEmpty() const
{
	return m_data.empty();
}

size_t AabbTreeData::getSize() const
{
	return m_data.size();
}

std::shared_ptr<AabbTreeData> AabbTreeData::takeLargerElements()
{
	std::shared_ptr<AabbTreeData> result(std::make_shared<AabbTreeData>());

	int axis;
	m_aabb.sizes().maxCoeff(&axis);
	double centerValue = m_aabb.center()(axis);
	/// HS-2015-03-20
	/// The new left and right aabb extents can probably be calculated here
	/// Only the separating axis extents would change the other two axis are unaffected
	/// #performance
	auto functor = [centerValue, axis](const Item & item)
	{
		return item.first.center()(axis) < centerValue;
	};
	auto split = std::partition(m_data.begin(), m_data.end(), functor);

	// In some cases all pieces may end up on one or the other side of the split, make this a noop
	if (split != m_data.begin() && split != m_data.end())
	{
		result->m_data.splice(result->m_data.end(), m_data, split, m_data.end());
		recalculateAabb();
		result->recalculateAabb();
	}

	return std::move(result);
}

void AabbTreeData::recalculateAabb()
{
	m_aabb.setNull();
	std::for_each(m_data.begin(), m_data.end(), [&](const Item & item)
	{
		m_aabb.extend(item.first);
	});
}

void AabbTreeData::getIntersections(const SurgSim::Math::Aabbd& aabb, std::list<size_t>* result) const
{
	std::for_each(m_data.begin(), m_data.end(), [&](const Item & item)
	{
		if (SurgSim::Math::doAabbIntersect(item.first, aabb))
		{
			result->push_back(item.second);
		}
	});
}

bool AabbTreeData::hasIntersections(const SurgSim::Math::Aabbd& aabb) const
{
	return SurgSim::Math::doAabbIntersect(m_aabb, aabb);
}


}
}


