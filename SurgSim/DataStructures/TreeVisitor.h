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

#ifndef SURGSIM_DATASTRUCTURES_TREEVISITOR_H
#define SURGSIM_DATASTRUCTURES_TREEVISITOR_H



namespace SurgSim
{
namespace DataStructures
{

class TreeNode;
class AabbTreeNode;

/// Abstract Class for visitors, this needs to be extended for other tree nodes when necessary
/// return false from handle() to abort traversal.
class TreeVisitor
{
public:

	/// Destructor
	virtual ~TreeVisitor()
	{

	}

	/// Handle TreeNode basic type,
	/// \return true To indicates that the visitor wishes to continue traversal, false if the visitor wants
	///              to abort traversal.
	virtual bool handle(TreeNode* node) = 0;

	/// Handle AabbTreeNode basic type
	virtual bool handle(AabbTreeNode* node)
	{
		return false;
	};
};

}
}

#endif
