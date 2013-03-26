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

#ifndef SURGSIM_DATA_STRUCTURES_TREE_H
#define SURGSIM_DATA_STRUCTURES_TREE_H

#include <memory>

namespace SurgSim
{

namespace DataStructures
{

class TreeNode;

class Tree
{
public:
	Tree();
	virtual ~Tree();

	void setRoot(std::shared_ptr<TreeNode> root)
	{
		m_root = root;
	}
	std::shared_ptr<TreeNode> getRoot() const
	{
		return m_root;
	}

	bool operator==(const Tree& tree) const;

	bool operator!=(const Tree& tree) const;

protected:
	virtual bool isEqual(const Tree& tree) const;

private:
	std::shared_ptr<TreeNode> m_root;

};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATA_STRUCTURES_TREE_H
