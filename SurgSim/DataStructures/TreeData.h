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

#ifndef SURGSIM_DATASTRUCTURES_TREEDATA_H
#define SURGSIM_DATASTRUCTURES_TREEDATA_H

#include <typeinfo>

namespace SurgSim
{

namespace DataStructures
{

/// Abstract base class for data stored in a Tree. Each TreeNode has a pointer to a TreeData object.
/// \sa TreeNode Tree
class TreeData
{
public:

	/// Constructor
	TreeData();

	/// Destructor
	virtual ~TreeData();

	/// If the data are not of the same type, returns false;
	/// otherwise, compares with the implementation of isEqual(const TreeData&).
	/// \param data Other TreeData for comparison.
	/// \return true if the data are equal; otherwise, returns false.
	bool operator==(const TreeData& data) const;

	/// Returns true if the data are not equal; otherwise, returns false.
	/// If the data are not of the same type, returns false;
	/// otherwise, compares with the implementation of isEqual(const TreeData&).
	/// \param data Other TreeData for comparison.
	/// \return true if the data are equal; otherwise, returns false.
	bool operator!=(const TreeData& data) const;

	virtual void clear() = 0;

private:

	/// Returns true if the trees are equal; otherwise, returns false.
	/// Implement this method in derived classes to do the comparison.
	/// \param data Other TreeData for comparison.
	/// \return true if the data are equal; otherwise, returns false.
	virtual bool isEqual(const TreeData* data) const = 0;
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_TREEDATA_H
