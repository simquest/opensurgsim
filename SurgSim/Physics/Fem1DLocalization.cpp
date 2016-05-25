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

#include "SurgSim/Physics/Fem1DLocalization.h"

#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/FemElement.h"

namespace SurgSim
{

namespace Physics
{

Fem1DLocalization::Fem1DLocalization(
	std::shared_ptr<Representation> representation,
	const SurgSim::DataStructures::IndexedLocalCoordinate& localPosition) :
	FemLocalization(representation, localPosition)
{
}

Fem1DLocalization::~Fem1DLocalization()
{
}

bool Fem1DLocalization::isValidRepresentation(std::shared_ptr<Representation> representation)
{
	auto femRepresentation = std::dynamic_pointer_cast<Fem1DRepresentation>(representation);

	// Allows to reset the representation to nullptr ...
	return (femRepresentation != nullptr || representation == nullptr);
}

bool Fem1DLocalization::moveClosestTo(const Math::Vector3d& point, bool* hasReachedEnd)
{
	if (hasReachedEnd != nullptr)
	{
		*hasReachedEnd = false;
	}
	bool moved = false;

	auto femRepresentation = std::static_pointer_cast<FemRepresentation>(getRepresentation());
	auto& currentState = femRepresentation->getCurrentState();
	
	double closestDistance = std::numeric_limits<double>::max(), newDistance;
	Math::Vector3d closestPoint, newPoint;
	std::array<Math::Vector3d, 2> closestNodePositions;
	size_t closestNodeIndex = std::numeric_limits<size_t>::max();
	
	auto numFemElements = femRepresentation->getNumFemElements();
	for (size_t i = 0; i < numFemElements; ++i)
	{
		auto femElement = femRepresentation->getFemElement(i);
		const auto& nodeIds = femElement->getNodeIds();
		std::array<Math::Vector3d, 2> nodePositions = {currentState->getPosition(nodeIds[0]),
			currentState->getPosition(nodeIds[1])};

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

	if (closestNodeIndex < numFemElements)
	{
		Math::Vector2d bary;
		Math::barycentricCoordinates(closestPoint, closestNodePositions[0], closestNodePositions[1], &bary);
		auto position = getLocalPosition();
		position.index = closestNodeIndex;
		position.coordinate[0] = bary[0];
		position.coordinate[1] = bary[1];
		if (hasReachedEnd != nullptr)
		{
			*hasReachedEnd =
				(closestNodeIndex == 0 && std::abs(position.coordinate[0] - 1.0) < Math::Geometry::DistanceEpsilon) ||
				(closestNodeIndex == numFemElements - 1 && std::abs(position.coordinate[1] - 1.0) < Math::Geometry::DistanceEpsilon);
		}
		setLocalPosition(position);
		return true;
	}

	return false;

	//auto femElement = femRepresentation->getFemElement(position.index);
	//const auto& nodeIds = femElement->getNodeIds();
	//std::array<Math::Vector3d, 2> nodePositions = {femRepresentation->getCurrentState()->getPosition(nodeIds[0]),
	//	femRepresentation->getCurrentState()->getPosition(nodeIds[1])};

	//Math::Vector3d currentNodePosition =
	//	position.coordinate[0] * nodePositions[0] + position.coordinate[1] * nodePositions[1];
	//std::array<Math::Vector3d, 2> nodeDirection = {nodePositions[0] - currentNodePosition,
	//	nodePositions[1] - currentNodePosition};
	//Math::Vector3d direction = point - currentNodePosition;

	//int increment = -1;
	//if (nodeDirection[0].dot(direction) < nodeDirection[1].dot(direction))
	//{
	//	increment = 1;
	//}

	//// Moving toward nodeIds[i] till the closest point to 'point' is reached.
	//double currentDistanceToPoint = (currentNodePosition - point).norm();
	//Math::Vector3d currentPointOnLine, newPointOnLine;
	//Math::distancePointSegment(point, nodePositions[0], nodePositions[1], &newPointOnLine);
	//double newDistanceToPoint = (newPointOnLine - point).norm();
	//currentPointOnLine = newPointOnLine;

	//while (newDistanceToPoint < currentDistanceToPoint)
	//{
	//	moved = true;
	//	currentDistanceToPoint = newDistanceToPoint;
	//	currentPointOnLine = newPointOnLine;
	//	auto index = position.index + increment;
	//	if (index >= 0 && index < femRepresentation->getNumFemElements())
	//	{
	//		position.index = index;
	//		auto femElement = femRepresentation->getFemElement(position.index);
	//		const auto&  nodeIds = femElement->getNodeIds();
	//		std::array<Math::Vector3d, 2> nodePositionsLocal =
	//		{femRepresentation->getCurrentState()->getPosition(nodeIds[0]),
	//		femRepresentation->getCurrentState()->getPosition(nodeIds[1])};

	//		Math::distancePointSegment(point, nodePositionsLocal[0], nodePositionsLocal[1], &newPointOnLine);
	//		newDistanceToPoint = (newPointOnLine - point).norm();
	//		if (newDistanceToPoint >= currentDistanceToPoint)
	//		{
	//			position.index -= increment;
	//		}
	//		else
	//		{
	//			nodePositions[0] = nodePositionsLocal[0];
	//			nodePositions[1] = nodePositionsLocal[1];
	//		}
	//	}
	//	else
	//	{
	//		break;
	//	}
	//}

	//if (moved)
	//{
	//	Math::Vector2d bary;
	//	Math::barycentricCoordinates(currentPointOnLine, nodePositions[0], nodePositions[1], &bary);
	//	position.coordinate[0] = bary[0];
	//	position.coordinate[1] = bary[1];
	//	if (hasReachedEnd != nullptr)
	//	{
	//		*hasReachedEnd =
	//			(increment > 0 && position.index == femRepresentation->getNumFemElements() - 1 &&
	//			 std::abs(position.coordinate[1] - 1.0) < Math::Geometry::DistanceEpsilon) ||
	//			(position.index == 0 && std::abs(position.coordinate[0] - 1.0) < Math::Geometry::DistanceEpsilon);
	//	}
	//	setLocalPosition(position);
	//}

	//return moved;
}

} // namespace Physics

} // namespace SurgSim
