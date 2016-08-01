// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Collision/ElementContactFilter.h"

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Collision/CollisionPair.h"

namespace SurgSim
{
namespace Collision
{



SURGSIM_REGISTER(SurgSim::Framework::Component, ElementContactFilter, ElementContactFilter);

ElementContactFilter::ElementContactFilter(const std::string& name) : ContactFilter(name)
{

}


bool ElementContactFilter::doInitialize()
{
	return true;
}

bool ElementContactFilter::doWakeUp()
{
	return true;
}

void ElementContactFilter::addFilterElements(const std::shared_ptr<Collision::Representation>& rep,
		std::vector<size_t> indices)
{
	std::vector<size_t> result;
	std::sort(indices.begin(), indices.end());
	std::set_union(indices.begin(), indices.end(), m_data[rep.get()].begin(), m_data[rep.get()].end(),
				   std::back_inserter(result));

	m_data[rep.get()] = result;
}

void ElementContactFilter::setFilterElements(const std::shared_ptr<Collision::Representation>& rep,
		std::vector<size_t> indices)
{
	m_data.emplace(rep.get(), std::move(indices));
}

std::vector<size_t> ElementContactFilter::getFilterElements(
	const std::shared_ptr<Collision::Representation>& rep) const
{
	auto found = m_data.find(rep.get());
	if (found != m_data.end())
	{
		return found->second;
	}

	return std::vector<size_t>();
}

void ElementContactFilter::clearElements(const std::shared_ptr<Collision::Representation>& rep)
{
	m_data[rep.get()].clear();
}

void ElementContactFilter::doFilterContacts(const std::shared_ptr<Physics::PhysicsManagerState>&,
		const std::shared_ptr<CollisionPair>& pair)
{
	for (int i = 0; i < 2; ++i)
	{
		auto found = m_data.find(pairAt(pair->getRepresentations(), i).get());
		if (found != m_data.end() && !(*found).second.empty())
		{
			excecuteFilter(pair, i, (*found).second);
		}
	}
}

void ElementContactFilter::excecuteFilter(
	const std::shared_ptr<CollisionPair>& pair,
	size_t pairIndex,
	const std::vector<size_t>& filter)
{
	auto shapeType = pairAt(pair->getRepresentations(), pairIndex)->getShapeType();
	DataStructures::Location::Type locationType;

	if (shapeType == Math::SHAPE_TYPE_MESH || shapeType == Math::SHAPE_TYPE_SURFACEMESH)
	{
		locationType = DataStructures::Location::TRIANGLE;
	}
	else if (shapeType == Math::SHAPE_TYPE_SEGMENTMESH)
	{
		locationType = DataStructures::Location::ELEMENT;
	}

	pair->getContacts().erase(
		(std::remove_if(pair->getContacts().begin(), pair->getContacts().end(),
						[pairIndex, locationType, &filter](const std::shared_ptr<Collision::Contact>& contact)
	{
		return pairAt(contact->penetrationPoints, pairIndex).get(locationType).hasValue() &&
			   std::find(filter.begin(), filter.end(), pairAt(contact->penetrationPoints,
						 pairIndex).get(locationType).getValue().index) != filter.end();
	})), pair->getContacts().end());
}

}
}
