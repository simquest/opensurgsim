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

#ifndef SURGSIM_DATASTRUCTURES_AABBTREEDATA_H
#define SURGSIM_DATASTRUCTURES_AABBTREEDATA_H

#include "SurgSim/DataStructures/TreeData.h"

#include "SurgSim/Math/Aabb.h"

#include <utility>
#include <list>
#include <memory>

namespace SurgSim
{
namespace DataStructures
{

/// Internal class to hold a list of AABBs and their respective object ids, it can calculate the elements
/// that intersect with a given aabb each node in the AABB tree holds one of these.
class AabbTreeData : public TreeData
{
public:

	typedef std::pair<SurgSim::Math::Aabbd, size_t> Item;
	typedef std::list<Item> ItemList;

	/// Constructor
	AabbTreeData();

	/// Copy Constructor
	AabbTreeData(const AabbTreeData& data);

	/// Constructor with list of items
	explicit AabbTreeData(const ItemList& data);

	/// Constructor with moveable list of items
	explicit AabbTreeData(ItemList&& data);

	/// Destructor
	~AabbTreeData();


	/// Add an item to the data
	/// \param aabb the AABB of the item
	/// \param id an object identifier assigned by the user of this class
	void add(const SurgSim::Math::Aabbd& aabb, size_t id);

	/// \return the combined AABB of all the contained items
	const SurgSim::Math::Aabbd& getAabb() const;

	/// \return true when there are no items, false otherwise
	bool isEmpty() const;

	/// \return the number of items
	size_t getSize() const;

	/// Split the current items into two geometric halves, keep the first half and return a pointer to the second half.
	/// The split is done along the longest axis of the enclosing aabb, the center of this axis is the point where
	/// the split occurs. This object will keep items that have a smaller coordinate than the center, the result will
	/// receive all items that have a larger coordinate on the determined axis.
	/// \return AabbTreeData with the items to the right of the center of the longest axis.
	std::shared_ptr<AabbTreeData> takeLargerElements();

	/// Check whether there could be any intersections with a given bounding box.
	/// \param aabb bounding box to use for the intersection check.
	/// \return true if the given AABB intersects with the AABB of all contained items.
	bool hasIntersections(const SurgSim::Math::Aabbd& aabb) const;

	/// Check all items bounding boxes against the one passed as a parameter and append items that overlap
	/// to the list given as a parameter
	/// \param aabb the bounding box being queried
	/// \param [out] result list to be used for intersecting items
	void getIntersections(const SurgSim::Math::Aabbd& aabb, std::list<size_t>* result) const;

	ItemList& getData()
	{
		return m_data;
	}

	/// Recalculate the aabb of this class, in case items where updated
	void recalculateAabb();

private:

	bool isEqual(const TreeData* data) const override;

	/// AABB containing all items
	SurgSim::Math::Aabbd m_aabb;

	/// The items that were added to this list

	ItemList m_data;
};

}
}

#endif
