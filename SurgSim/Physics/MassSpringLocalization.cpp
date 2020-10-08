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

#include "SurgSim/Physics/MassSpringLocalization.h"

#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/MassSpring.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"


namespace SurgSim
{
namespace Physics
{
MassSpringLocalization::MassSpringLocalization()
{

}

MassSpringLocalization::MassSpringLocalization(
	std::shared_ptr<Representation> representation) :
	Localization()
{
	setRepresentation(representation);
}

MassSpringLocalization::~MassSpringLocalization()
{

}

void MassSpringLocalization::setLocalNode(size_t nodeID)
{
	SURGSIM_ASSERT(!m_position.hasValue()) <<
		"MassSpringLocalization cannot setLocalNode because it already has a local position.";
	m_nodeID = nodeID;
}

const DataStructures::OptionalValue<size_t>& MassSpringLocalization::getLocalNode() const
{
	return m_nodeID;
}

void MassSpringLocalization::setLocalPosition(const SurgSim::DataStructures::IndexedLocalCoordinate & localPosition)
{
	auto massSpringRepresentation = std::static_pointer_cast<MassSpringRepresentation>(getRepresentation());
	SURGSIM_ASSERT(massSpringRepresentation != nullptr) << "MassSpringRepresentation is null, it was probably not" <<
		" initialized";
	SURGSIM_ASSERT(!m_nodeID.hasValue()) <<
		"MassSpringLocalization cannot setLocalPosition because it already has a local node.";

	SURGSIM_ASSERT(massSpringRepresentation->isValidCoordinate(localPosition))
		<< "IndexedLocalCoordinate is invalid for Representation " << getRepresentation()->getName();

	m_position = localPosition;
}

const DataStructures::OptionalValue<DataStructures::IndexedLocalCoordinate>&
MassSpringLocalization::getLocalPosition() const
{
	return m_position;
}

SurgSim::Math::Vector3d MassSpringLocalization::doCalculatePosition(double time) const
{
	auto massSpringRepresentation = std::static_pointer_cast<MassSpringRepresentation>(getRepresentation());
	SURGSIM_ASSERT(massSpringRepresentation != nullptr) << "MassSpringRepresentation is null, it was probably not" <<
		" initialized";

	Math::Vector3d currentPoint;
	Math::Vector3d previousPoint;
	if (m_nodeID.hasValue())
	{
		currentPoint = massSpringRepresentation->getCurrentState()->getPosition(m_nodeID.getValue());
		previousPoint = massSpringRepresentation->getPreviousState()->getPosition(m_nodeID.getValue());
	}
	else if (m_position.hasValue())
	{
		previousPoint =
			massSpringRepresentation->computeCartesianCoordinate(*massSpringRepresentation->getPreviousState(),
			m_position.getValue());
		currentPoint =
			massSpringRepresentation->computeCartesianCoordinate(*massSpringRepresentation->getCurrentState(),
			m_position.getValue());
	}
	else
	{
		SURGSIM_FAILURE() << "Cannot calculate position of a MassSpringLocalization that has neither index nor " <<
			"local position";
	}

	if (time <= std::numeric_limits<double>::epsilon())
	{
		return previousPoint;
	}
	else if (time >= 1.0 - std::numeric_limits<double>::epsilon())
	{
		return currentPoint;
	}

	return SurgSim::Math::interpolate(previousPoint, currentPoint, time);
}

SurgSim::Math::Vector3d MassSpringLocalization::doCalculateVelocity(double time) const
{
	auto massSpringRepresentation =	std::static_pointer_cast<MassSpringRepresentation>(getRepresentation());
	SURGSIM_ASSERT(massSpringRepresentation != nullptr) << "MassSpringRepresentation is null, it was probably not" <<
		" initialized";

	Math::Vector3d currentVelocity;
	Math::Vector3d previousVelocity;
	if (m_nodeID.hasValue())
	{
		currentVelocity = massSpringRepresentation->getCurrentState()->getVelocity(m_nodeID.getValue());
		previousVelocity = massSpringRepresentation->getPreviousState()->getVelocity(m_nodeID.getValue());
	}
	else if (m_position.hasValue())
	{
		previousVelocity =
			massSpringRepresentation->computeCartesianCoordinate(*massSpringRepresentation->getPreviousState(),
				m_position.getValue());
		currentVelocity =
			massSpringRepresentation->computeCartesianCoordinate(*massSpringRepresentation->getCurrentState(),
				m_position.getValue());
	}
	else
	{
		SURGSIM_FAILURE() << "Cannot calculate position of a MassSpringLocalization that has neither index nor " <<
			"local position";
	}

	if (time <= std::numeric_limits<double>::epsilon())
	{
		return previousVelocity;
	}
	else if (time >= 1.0 - std::numeric_limits<double>::epsilon())
	{
		return currentVelocity;
	}

	return SurgSim::Math::interpolate(previousVelocity, currentVelocity, time);
}

bool MassSpringLocalization::isValidRepresentation(std::shared_ptr<Representation> representation)
{

	std::shared_ptr<MassSpringRepresentation> massSpringRepresentation =
		std::dynamic_pointer_cast<MassSpringRepresentation>(representation);

	// Allows to reset the representation to nullptr ...
	return (massSpringRepresentation != nullptr || representation == nullptr);
}

bool MassSpringLocalization::moveClosestTo(const Math::Vector3d& point, bool* hasReachedEnd)
{
	if (hasReachedEnd != nullptr)
	{
		*hasReachedEnd = false;
	}

	auto massSpringRepresentation = std::static_pointer_cast<MassSpringRepresentation>(getRepresentation());
	auto& currentState = massSpringRepresentation->getCurrentState();
	auto massSpring = massSpringRepresentation->getMassSpring();
	SURGSIM_ASSERT(massSpring != nullptr);

	double closestDistance = std::numeric_limits<double>::max(), newDistance;
	Math::Vector3d closestPoint, newPoint;
	std::array<Math::Vector3d, 2> closestNodePositions;
	size_t closestNodeIndex = std::numeric_limits<size_t>::max();

	auto numElements = massSpring->getNumElements();
	for (size_t i = 0; i < numElements; ++i)
	{
		const auto& nodeIds = massSpring->getNodeIds(i);
		SURGSIM_ASSERT(nodeIds.size() == 2) <<
			"MassSpringLocalization::moveClosestTo is only defined for a MassSpring that has 2 nodes per element." <<
			"\nInstead it got " << nodeIds.size() << " nodes.";
		std::array<Math::Vector3d, 2> nodePositions = { currentState->getPosition(nodeIds[0]),
													   currentState->getPosition(nodeIds[1])
		};

		Math::distancePointSegment(point, nodePositions[0], nodePositions[1], &newPoint);
		newDistance = (newPoint - point).norm();
		if (newDistance < closestDistance)
		{
			closestNodeIndex = i;
			closestDistance = newDistance;
			closestPoint = newPoint;
			closestNodePositions[0] = nodePositions[0];
			closestNodePositions[1] = nodePositions[1];
		}
	}

	if (closestNodeIndex < numElements)
	{
		Math::Vector2d bary;
		Math::barycentricCoordinates(closestPoint, closestNodePositions[0], closestNodePositions[1], &bary);
		m_nodeID.invalidate(); // In case the localization had been using the local node.
		DataStructures::IndexedLocalCoordinate position(closestNodeIndex, bary);
		if (hasReachedEnd != nullptr)
		{
			*hasReachedEnd =
				(closestNodeIndex == 0 && std::abs(position.coordinate[0] - 1.0) < Math::Geometry::DistanceEpsilon) ||
				(closestNodeIndex == numElements - 1 &&
					std::abs(position.coordinate[1] - 1.0) < Math::Geometry::DistanceEpsilon);
		}
		setLocalPosition(position);
		return true;
	}

	return false;
}

std::shared_ptr<Localization> MassSpringLocalization::doCopy() const
{
	auto localization = std::make_shared<MassSpringLocalization>();
	localization->setRepresentation(getRepresentation());
	if (getLocalNode().hasValue())
	{
		localization->setLocalNode(getLocalNode().getValue());
	}
	if (getLocalPosition().hasValue())
	{
		localization->setLocalPosition(getLocalPosition().getValue());
	}
	return localization;
}

}; // Physics
}; // SurgSim


