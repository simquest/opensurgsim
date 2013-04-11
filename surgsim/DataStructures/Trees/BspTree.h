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

#ifndef SURGSIM_BSP_TREE_H
#define SURGSIM_BSP_TREE_H

#include "Tree.h"

#include "BspTreeNode.h"

namespace SurgSim
{

template <unsigned int D>
class BspTree : public Tree
{
public:
	virtual ~BspTree() 
	{
	}

protected:
	BspTree() : Tree()
	{
	}

	virtual bool isEqual(const Tree& tree) const
	{
		const BspTree& bspTree = static_cast<const BspTree&>(tree);
		return Tree::isEqual(tree);
	}

private:

};

};  // namespace SurgSim

#endif  // BSP_TREE_H
