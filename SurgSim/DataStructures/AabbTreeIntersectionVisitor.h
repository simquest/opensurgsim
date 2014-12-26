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

/// Visitor class to collect the items that intersect with a given bounding box
class AabbTreeIntersectionVisitor : public TreeVisitor
{
public:

	/// Constructor
	AabbTreeIntersectionVisitor();

	/// Constructor
	/// \param aabb the bounding box to be used.
	explicit AabbTreeIntersectionVisitor(const SurgSim::Math::Aabbd& aabb);

	/// Destructor
	virtual ~AabbTreeIntersectionVisitor();

	bool handle(TreeNode* node) override;

	bool handle(AabbTreeNode* node) override;

	/// \return true if the visitor has found intersections
	bool hasIntersections() const;

	/// Resets the data in the tree
	void reset();

	/// \return the bounding box to be used for the test.
	SurgSim::Math::Aabbd getAabb() const;

	/// Sets a new bounding box, will also call reset()
	/// \param aabb The new bounding box.
	void setAabb(const SurgSim::Math::Aabbd& aabb);

	/// \return a reference to the found intersections.
	const std::list<size_t>& getIntersections() const;

private:

	/// List of ids found for intersections
	std::list<size_t> m_intersections;

	/// Bounding box used for intersection test
	SurgSim::Math::Aabbd m_aabb;
};

}
}

#endif
