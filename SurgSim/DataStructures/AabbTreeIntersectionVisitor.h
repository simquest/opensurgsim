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

#ifndef SURGSIM_DATASTRUCTURES_AABBTREEINTERSECTIONVISITOR_H
#define SURGSIM_DATASTRUCTURES_AABBTREEINTERSECTIONVISITOR_H

#include "SurgSim/DataStructures/TreeVisitor.h"
#include "SurgSim/Math/Aabb.h"

#include <list>

namespace SurgSim
{
namespace DataStructures
{

class AabbTreeIntersectionVisitor : public TreeVisitor
{
public:
	AabbTreeIntersectionVisitor();

	explicit AabbTreeIntersectionVisitor(const SurgSim::Math::Aabbd& aabb);

	virtual ~AabbTreeIntersectionVisitor();

	virtual bool handle(TreeNode* node) override;

	virtual bool handle(AabbTreeNode* node) override;

	bool hasIntersections();

	void reset();

	SurgSim::Math::Aabbd getAabb() const;

	void setAabb(SurgSim::Math::Aabbd val);

	const std::list<size_t>& getIntersections() const;

private:

	std::list<size_t> m_intersections;
	SurgSim::Math::Aabbd m_aabb;
};

}
}

#endif
