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
#include "SurgSim/Framework/FrameworkConvert.h"

namespace SurgSim
{
namespace Collision
{



SURGSIM_REGISTER(SurgSim::Framework::Component, ElementContactFilter, ElementContactFilter);

ElementContactFilter::ElementContactFilter(const std::string& name) : ContactFilter(name),
	m_logger(SurgSim::Framework::Logger::getLogger("Collision/ElementContactFilter"))
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(ElementContactFilter, std::shared_ptr<Framework::Component>, Representation,
									  getRepresentation, setRepresentation);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(ElementContactFilter, std::vector<size_t>, FilterElements,
									  getFilterElements, setFilterElements);

}


bool ElementContactFilter::doInitialize()
{
	return true;
}

bool ElementContactFilter::doWakeUp()
{
	bool valid = false;
	SURGSIM_LOG_IF(m_representation == nullptr, m_logger, WARNING)
			<< "No representation for filtering on " << getFullName();
	if (m_representation != nullptr)
	{
		auto type = m_representation->getShapeType();
		if (type == SurgSim::Math::SHAPE_TYPE_MESH || type == SurgSim::Math::SHAPE_TYPE_SEGMENTMESH ||
			type == SurgSim::Math::SHAPE_TYPE_SURFACEMESH)
		{
			valid = true;
		}
		else
		{
			SURGSIM_LOG_WARNING(m_logger)
					<< "ElementContactFilter " << getFullName()
					<< "invalid mesh structure for collision shape, current shape is " << type << std::endl
					<< "Should be one of " << SurgSim::Math::SHAPE_TYPE_MESH << ", "
					<< SurgSim::Math::SHAPE_TYPE_SURFACEMESH << ", or "
					<< SurgSim::Math::SHAPE_TYPE_SEGMENTMESH;
		}
	}

	return valid;
}

void ElementContactFilter::setFilterElements(const std::vector<size_t>& indices)
{
	m_writeBuffer.set(indices);
}

std::vector<size_t> ElementContactFilter::getFilterElements() const
{
	std::vector<size_t> result;
	m_writeBuffer.get(&result);
	return result;
}

void ElementContactFilter::clearFilterElements()
{
	m_writeBuffer.set(std::vector<size_t>());
}

void ElementContactFilter::setRepresentation(const std::shared_ptr<SurgSim::Framework::Component>& val)
{
	SURGSIM_ASSERT(!isAwake()) << "Can't set representation after waking up on " << getFullName();
	m_representation = Framework::checkAndConvert<SurgSim::Collision::Representation>(
						   val, "SurgSim::Collision::Representation");
}

std::shared_ptr<SurgSim::Collision::Representation> ElementContactFilter::getRepresentation() const
{
	return m_representation;
}

void ElementContactFilter::doFilterContacts(const std::shared_ptr<Physics::PhysicsManagerState>&,
		const std::shared_ptr<CollisionPair>& pair)
{
	if (m_indices.empty())
	{
		return;
	}

	for (int i = 0; i < 2; ++i)
	{
		if (pairAt(pair->getRepresentations(), i).get() == getRepresentation().get())
		{
			excecuteFilter(pair, i, m_indices);
		}
	}
}

void ElementContactFilter::doUpdate(double dt)
{
	m_writeBuffer.tryGetChanged(&m_indices);
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
	else
	{
		return;
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
