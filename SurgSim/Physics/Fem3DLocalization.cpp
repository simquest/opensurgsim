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

#include "SurgSim/Physics/Fem3DLocalization.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FemElement.h"

namespace SurgSim
{

namespace Physics
{

Fem3DLocalization::Fem3DLocalization(
	std::shared_ptr<Representation> representation,
	const SurgSim::DataStructures::IndexedLocalCoordinate& localPosition) :
	Localization()
{
	setRepresentation(representation);
	setLocalPosition(localPosition);
}

Fem3DLocalization::~Fem3DLocalization()
{

}

void Fem3DLocalization::setLocalPosition(
	const SurgSim::DataStructures::IndexedLocalCoordinate& localPosition)
{
	auto femRepresentation = std::static_pointer_cast<Fem3DRepresentation>(getRepresentation());

	SURGSIM_ASSERT(femRepresentation != nullptr) << "FemRepresentation is null, it was probably not" <<
		" initialized";

	SURGSIM_ASSERT(femRepresentation->isValidCoordinate(localPosition))
		<< "IndexedLocalCoordinate is invalid for Representation " << getRepresentation()->getName();

	m_position = localPosition;
}

const SurgSim::DataStructures::IndexedLocalCoordinate& Fem3DLocalization::getLocalPosition() const
{
	return m_position;
}

SurgSim::Math::Vector3d Fem3DLocalization::doCalculatePosition(double time)
{
	using SurgSim::Math::Vector3d;

	auto femRepresentation = std::static_pointer_cast<Fem3DRepresentation>(getRepresentation());

	SURGSIM_ASSERT(femRepresentation != nullptr) << "FemRepresentation is null, it was probably not" <<
		" initialized";

	std::shared_ptr<FemElement> femElement = femRepresentation->getFemElement(m_position.index);
	const Vector3d currentPosition = femElement->computeCartesianCoordinate(*femRepresentation->getCurrentState(),
		m_position.coordinate);
	const Vector3d previousPosition = femElement->computeCartesianCoordinate(*femRepresentation->getPreviousState(),
		m_position.coordinate);

	if (time == 0.0)
	{
		return previousPosition;
	}
	else if (time == 1.0)
	{
		return currentPosition;
	}

	return previousPosition + time * (currentPosition - previousPosition);
}

bool Fem3DLocalization::isValidRepresentation(std::shared_ptr<Representation> representation)
{
	auto femRepresentation = std::dynamic_pointer_cast<Fem3DRepresentation>(representation);

	// Allows to reset the representation to nullptr ...
	return (femRepresentation != nullptr || representation == nullptr);
}

} // namespace Physics

} // namespace SurgSim
