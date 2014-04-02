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

#include <SurgSim/DataStructures/AabbTreeNode.h>
#include <SurgSim/DataStructures/AabbTreeData.h>

#include <SurgSim/Framework/Assert.h>

namespace SurgSim
{
namespace DataStructures
{

AabbTreeNode::AabbTreeNode()
{
}

AabbTreeNode::~AabbTreeNode()
{

}

void AabbTreeNode::splitNode()
{
	auto leftData = std::static_pointer_cast<AabbTreeData>(getData());
	if (leftData->getSize() > 0)
	{
		std::shared_ptr<AabbTreeData> rightData = leftData->split();
		if (getNumChildren() != 2)
		{
			std::shared_ptr<TreeNode> child = std::make_shared<AabbTreeNode>();
			addChild(std::move(child));

			child = std::make_shared<AabbTreeNode>();
			addChild(std::move(child));
		}
		m_aabd = leftData->getAabb();
		m_aabd.extend(rightData->getAabb());
		getChild(0)->setData(std::move(leftData));
		getChild(1)->setData(std::move(rightData));
		setData(nullptr);
	}
}

const SurgSim::Math::Aabbd& AabbTreeNode::getAabb() const
{
	auto data = std::static_pointer_cast<AabbTreeData>(getData());
	if (data == nullptr)
	{
		return m_aabd;
	}
	else
	{
		return data->getAabb();
	}
}

void AabbTreeNode::addData(const SurgSim::Math::Aabbd& aabb, size_t id, size_t maxNodeData)
{

	if (getNumChildren() > 0)
	{
		for (size_t i = 0; i < getNumChildren(); ++i)
		{
			auto childData = std::static_pointer_cast<AabbTreeNode>(getChild(i));
			if (SurgSim::Math::doAabbIntersect(childData->getAabb(), aabb))
			{
				childData->addData(aabb, id, maxNodeData);
			}
		}
	}
	else
	{
		auto data = std::static_pointer_cast<AabbTreeData>(getData());
		if (data == nullptr)
		{
			data = std::make_shared<AabbTreeData>();
			setData(data);
		}
		data->add(aabb, id);
		if (maxNodeData > 0 && data->getSize() > maxNodeData)
		{
			splitNode();
		}
	}
}

}
}

