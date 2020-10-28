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

#include "SurgSim/Physics/Fem2DLocalization.h"

#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"
#include "SurgSim/Physics/FemElement.h"

namespace SurgSim
{

namespace Physics
{

Fem2DLocalization::Fem2DLocalization(
	std::shared_ptr<Representation> representation,
	const SurgSim::DataStructures::IndexedLocalCoordinate& localPosition) :
	FemLocalization(representation, localPosition)
{
}

Fem2DLocalization::~Fem2DLocalization()
{
}

bool Fem2DLocalization::isValidRepresentation(std::shared_ptr<Representation> representation)
{
	auto femRepresentation = std::dynamic_pointer_cast<Fem2DRepresentation>(representation);

	// Allows to reset the representation to nullptr ...
	return (femRepresentation != nullptr || representation == nullptr);
}

Math::RigidTransform3d Fem2DLocalization::getElementPose()
{
	auto femRepresentation = std::static_pointer_cast<Fem2DRepresentation>(getRepresentation());
	auto position = getLocalPosition();
	auto femElement = femRepresentation->getFemElement(getLocalPosition().index);
	const auto& nodeIds = femElement->getNodeIds();
	std::array<Math::Vector3d, 3> nodePositions =
		{femRepresentation->getCurrentState()->getPosition(nodeIds[0]),
		 femRepresentation->getCurrentState()->getPosition(nodeIds[1]),
		 femRepresentation->getCurrentState()->getPosition(nodeIds[2])};

	Math::Vector3d edge, normal, binormal;
	edge = (nodePositions[1] - nodePositions[0]).normalized();
	normal = (nodePositions[2] - nodePositions[0]).cross(edge).normalized();
	binormal = edge.cross(normal);
	Math::Matrix33d rotation;
	rotation << edge, normal, binormal;

	return Math::makeRigidTransform(rotation, (nodePositions[0] + nodePositions[1] + nodePositions[2]) / 3.0);
}

std::shared_ptr<Localization> Fem2DLocalization::doCopy() const
{
	return std::make_shared<Fem2DLocalization>(getRepresentation(), getLocalPosition());
}

} // namespace Physics

} // namespace SurgSim
