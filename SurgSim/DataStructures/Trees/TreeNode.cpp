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

#include "TreeNode.h"

#include <SurgSim/DataStructures/Trees/TreeData.h>

#include <typeinfo>

using SurgSim::DataStructures::TreeNode;

TreeNode::TreeNode() : m_data(nullptr) 
{
}
TreeNode::~TreeNode()
{
}

bool TreeNode::operator==(const TreeNode& node) const
{
	return (typeid(*this) == typeid(node)) && isEqual(node);
}

bool TreeNode::operator!=(const TreeNode& node) const
{
	return (typeid(*this) != typeid(node)) || ! isEqual(node);
}

bool TreeNode::isEqual(const TreeNode& node) const
{
	if ( (m_data != nullptr && node.m_data == nullptr) || (m_data == nullptr && node.m_data != nullptr) )
	{
		return false;
	}
	if (m_data != nullptr && node.m_data != nullptr)
	{
		return *m_data == *node.m_data;
	}
	return true;
}