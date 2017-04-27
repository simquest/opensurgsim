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

#include "SurgSim/DataStructures/AabbTreeNode.h"
#include "SurgSim/DataStructures/AabbTreeData.h"

#include "SurgSim/Framework/Log.h"

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

void AabbTreeNode::splitNode(size_t maxNodeData)
{
	auto leftData = std::static_pointer_cast<AabbTreeData>(getData());
	if (leftData->getSize() > maxNodeData)
	{
		std::shared_ptr<AabbTreeData> rightData = leftData->takeLargerElements();
		std::shared_ptr<AabbTreeNode> leftChild;
		std::shared_ptr<AabbTreeNode> rightChild;

		// Early exit, takeLargerElements() may not be able to split the list, in this
		// case we abort the splitNode.
		if (leftData->getSize() == 0 || rightData->getSize() == 0)
		{
			auto count = std::max(leftData->getSize(), rightData->getSize());

			if (maxNodeData > 0 && count > 3 * maxNodeData)
			{
				SURGSIM_LOG_ONCE(Framework::Logger::getLogger("DataStructures/AabbTreeNode"), WARNING)
						<< "The aabb tree build process encountered some items that could not be split anymore "
						<< "this may cause suboptimal behavior when querying the aabb tree.";
			}
			return;
		}

		if (getNumChildren() != 2)
		{
			leftChild = std::make_shared<AabbTreeNode>();
			addChild(leftChild);

			rightChild = std::make_shared<AabbTreeNode>();
			addChild(rightChild);
		}
		else
		{
			leftChild = std::static_pointer_cast<AabbTreeNode>(getChild(0));
			rightChild = std::static_pointer_cast<AabbTreeNode>(getChild(1));
		}

		// Update the local aabb
		// The axis won't change after it has been split
		m_aabb = leftData->getAabb();
		m_aabb.extend(rightData->getAabb());
		m_aabb.sizes().maxCoeff(&m_axis);

		setData(nullptr);

		size_t leftCount = leftData->getSize();
		size_t rightCount = rightData->getSize();

		leftChild->setData(std::move(leftData));
		if (maxNodeData > 0 && leftCount > maxNodeData)
		{
			leftChild->splitNode(maxNodeData);
		}

		rightChild->setData(std::move(rightData));
		if (maxNodeData > 0 && rightCount > maxNodeData)
		{
			rightChild->splitNode(maxNodeData);
		}
	}
}

const SurgSim::Math::Aabbd& AabbTreeNode::getAabb() const
{
	auto data = std::static_pointer_cast<AabbTreeData>(getData());
	if (data == nullptr)
	{
		return m_aabb;
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
		size_t childIndex = (aabb.center()(m_axis) < m_aabb.center()(m_axis)) ? 0 : 1;
		auto childNode = std::static_pointer_cast<AabbTreeNode>(getChild(childIndex));
		childNode->addData(aabb, id, maxNodeData);
		m_aabb.extend(aabb);
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

void AabbTreeNode::setData(const AabbTreeData::ItemList& items, size_t maxNodeData)
{
	SURGSIM_ASSERT(getNumChildren() == 0) << "Can't call setData on a node that already has nodes";
	SURGSIM_ASSERT(getData() == nullptr) << "Can't call setData on a node that already has data.";

	auto data = std::make_shared<AabbTreeData>(items);
	setData(data);
	splitNode(maxNodeData);
}

void AabbTreeNode::setData(AabbTreeData::ItemList&& items, size_t maxNodeData)
{
	SURGSIM_ASSERT(getNumChildren() == 0) << "Can't call setData on a node that already has nodes";
	SURGSIM_ASSERT(getData() == nullptr) << "Can't call setData on a node that already has data.";

	auto data = std::make_shared<AabbTreeData>(std::move(items));
	setData(data);
	splitNode(maxNodeData);
}

bool AabbTreeNode::doAccept(TreeVisitor* visitor)
{
	return visitor->handle(this);
}

void AabbTreeNode::getIntersections(const SurgSim::Math::Aabbd& aabb, std::list<size_t>* result)
{
	auto data = std::static_pointer_cast<AabbTreeData>(getData());
	SURGSIM_ASSERT(data != nullptr) << "AabbTreeNode data is nullptr.";
	data->getIntersections(aabb, result);
}

}
}

