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

#include "SurgSim/DataStructures/AabbTreeIntersectionVisitor.h"

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/DataStructures/AabbTreeNode.h"
#include "SurgSim/Math/Aabb.h"

namespace SurgSim
{
namespace DataStructures
{
AabbTreeIntersectionVisitor::AabbTreeIntersectionVisitor()
{

}

AabbTreeIntersectionVisitor::AabbTreeIntersectionVisitor(const SurgSim::Math::Aabbd& aabb) :
	m_aabb(aabb)
{

}

AabbTreeIntersectionVisitor::~AabbTreeIntersectionVisitor()
{

}

bool AabbTreeIntersectionVisitor::handle(TreeNode* node)
{
	SURGSIM_FAILURE() << "Can only handle AabbTreeNodes, this is of a different type.";
	return false;
}

bool AabbTreeIntersectionVisitor::handle(AabbTreeNode* node)
{
	bool result = false;

	if (node->getNumChildren() == 0)
	{
		// On a leaf node, perform the intersections
		node->getIntersections(m_aabb, &m_intersections);
	}
	else
	{
		// On non leaf nodes check if we have an intersection with the common aabb
		result = SurgSim::Math::doAabbIntersect(m_aabb, node->getAabb());
	}

	return result;
}


void AabbTreeIntersectionVisitor::reset()
{
	m_intersections.clear();
}

SurgSim::Math::Aabbd AabbTreeIntersectionVisitor::getAabb() const
{
	return m_aabb;
}


void AabbTreeIntersectionVisitor::setAabb(const SurgSim::Math::Aabbd& aabb)
{
	m_aabb = aabb;
	reset();
}


const std::vector<size_t>& AabbTreeIntersectionVisitor::getIntersections() const
{
	return m_intersections;
}

bool AabbTreeIntersectionVisitor::hasIntersections() const
{
	return !m_intersections.empty();
}

}
}

