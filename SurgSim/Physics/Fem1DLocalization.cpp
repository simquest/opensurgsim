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

void Fem1DLocalization::moveClosestTo(const Math::Vector3d& point, bool *hasReachedEnd)
{
	auto femRepresentation = std::static_pointer_cast<FemRepresentation>(getRepresentation());
	auto position = getLocalPosition();
	auto femElement = femRepresentation->getFemElement(position.index);
	auto nodeIds = femElement->getNodeIds();
	std::vector<Math::Vector3d> nodePositions;
	for (auto nodeId : nodeIds)
	{
		nodePositions.push_back(femRepresentation->getCurrentState()->getPosition(nodeId));
	}

	auto currentNodePosition = position.coordinate[0] * nodePositions[0] + position.coordinate[1] * nodePositions[1];
	std::array<Math::Vector3d, 2> nodeDirection = {nodePositions[0] - currentNodePosition,
		nodePositions[1] - currentNodePosition};
	Math::Vector3d direction = point - currentNodePosition;

	size_t i = 0, j = 1;
	int increment[2] = {-1, 1};
	size_t elementEnds[2] = {0, femRepresentation->getNumFemElements() - 1};
	if (nodeDirection[i].dot(direction) < nodeDirection[j].dot(direction))
	{
		i = 1;
		j = 0;
	}

	// Moving toward nodeIds[i] till the closest point to 'point' is reached.
	double currentDistanceToPoint = (currentNodePosition - point).norm();
	Math::Vector3d currentPointOnLine, newPointOnLine;
	Math::distancePointSegment(point, nodePositions[i], nodePositions[j], &newPointOnLine);
	double newDistanceToPoint = (newPointOnLine - point).norm();
	currentPointOnLine = newPointOnLine;
	bool setCoordinates = false;

	while (newDistanceToPoint < currentDistanceToPoint)
	{
		setCoordinates = true;
		currentDistanceToPoint = newDistanceToPoint;
		currentPointOnLine = newPointOnLine;
		auto index = position.index + increment[i];
		if (index >= 0 && index < femRepresentation->getNumFemElements())
		{
			position.index = index;
			auto femElement = femRepresentation->getFemElement(position.index);
			auto nodeIds = femElement->getNodeIds();
			std::vector<Math::Vector3d> nodePositionsLocal;
			for (auto nodeId : nodeIds)
			{
				nodePositionsLocal.push_back(femRepresentation->getCurrentState()->getPosition(nodeId));
			}
			Math::distancePointSegment(point, nodePositionsLocal[i], nodePositionsLocal[j], &newPointOnLine);
			newDistanceToPoint = (newPointOnLine - point).norm();
			if (newDistanceToPoint >= currentDistanceToPoint)
			{
				position.index -= increment[i];
			}
			else
			{
				nodePositions[i] = nodePositionsLocal[i];
				nodePositions[j] = nodePositionsLocal[j];
			}
		}
		else
		{
			break;
		}
	}

	if (setCoordinates)
	{
		Math::Vector2d bary;
		Math::barycentricCoordinates(currentPointOnLine, nodePositions[i], nodePositions[j], &bary);
		position.coordinate[i] = bary[0];
		position.coordinate[j] = bary[1];
		if (position.index == elementEnds[i])
		{
			*hasReachedEnd = std::abs(position.coordinate[i] - 1.0) < Math::Geometry::DistanceEpsilon;
		}
		setLocalPosition(position);
	}
}

} // namespace Physics

} // namespace SurgSim
