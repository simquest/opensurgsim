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

#ifndef SURGSIM_DATA_STRUCTURES_AABB_TREE_DATA_H
#define SURGSIM_DATA_STRUCTURES_AABB_TREE_DATA_H

#include "TreeData.h"

#include <SurgSim/Math/Vector.h>

namespace SurgSim
{

namespace DataStructures
{

class AabbTreeData : public TreeData
{

public:
	AabbTreeData();
	AabbTreeData(const SurgSim::Math::Vector3d& minimum, const SurgSim::Math::Vector3d& maximum);
	virtual ~AabbTreeData();

	void reset()
	{
		m_minimum.setConstant(-std::numeric_limits<double>::max());
		m_maximum.setConstant(std::numeric_limits<double>::max());
	}

	void addPoint(const SurgSim::Math::Vector3d& position)
	{
		for (int i = 0; i < 3; ++i)
		{
			m_minimum[i] = std::min(m_minimum[i], position[i]);
			m_maximum[i] = std::max(m_maximum[i], position[i]);
		}
	}

	void expandToContain(const AabbTreeData& aabb)
	{
		for (int i = 0; i < 3; ++i)
		{
			m_minimum[i] = std::min(m_minimum[i], aabb.m_minimum[i]);
			m_maximum[i] = std::max(m_maximum[i], aabb.m_maximum[i]);
		}
	}

private:
	virtual bool isEqual(const TreeData& data) const
	{
		const AabbTreeData& aabbTreeData = static_cast<const AabbTreeData&>(data);
		return (m_minimum == aabbTreeData.m_minimum) && (m_maximum == aabbTreeData.m_maximum);
	}

	SurgSim::Math::Vector3d m_minimum;
	SurgSim::Math::Vector3d m_maximum;
};

};  // namespace DataStructures
};  // namespace SurgSim

#endif  // SURGSIM_DATA_STRUCTURES_AABB_TREE_DATA_H
