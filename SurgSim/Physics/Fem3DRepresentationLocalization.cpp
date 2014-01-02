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

#include "SurgSim/Physics/Fem3DRepresentationLocalization.h"
#include "SurgSim/Physics/FemRepresentation.h"
#include "SurgSim/Math/Vector.h"

#include "SurgSim/Physics/Fem3DRepresentation.h"

namespace SurgSim
{

namespace Physics
{

Fem3DRepresentationLocalization::Fem3DRepresentationLocalization()
{

}

Fem3DRepresentationLocalization::Fem3DRepresentationLocalization(std::shared_ptr<Representation> representation) :
	Localization()
{
	setRepresentation(representation);
}

Fem3DRepresentationLocalization::~Fem3DRepresentationLocalization()
{

}

void Fem3DRepresentationLocalization::setLocalPosition(const FemRepresentationCoordinate& p)
{
	m_position = p;
}

const FemRepresentationCoordinate& Fem3DRepresentationLocalization::getLocalPosition() const
{
	return m_position;
}

SurgSim::Math::Vector3d Fem3DRepresentationLocalization::doCalculatePosition(double time)
{
	auto femRepresentation = std::static_pointer_cast<Fem3DRepresentation>(getRepresentation());

	SURGSIM_ASSERT(femRepresentation != nullptr) << "FemRepresentation is null, it was probably not" <<
		" initialized";

	std::shared_ptr<FemElement> femElement = femRepresentation->getFemElement(m_position.elementId);
	const std::shared_ptr<DeformableRepresentationState> previousState = femRepresentation->getPreviousState();
	const std::shared_ptr<DeformableRepresentationState> currentState = femRepresentation->getCurrentState();

	if (time == 0.0)
	{
		return femElement->computeCartesianCoordinate(*previousState, m_position.barycentricCoordinate);
	}
	else if (time == 1.0)
	{
		return femElement->computeCartesianCoordinate(*currentState, m_position.barycentricCoordinate);
	}

	const SurgSim::Math::Vector& currentPosition = femElement->computeCartesianCoordinate(*previousState,
		m_position.barycentricCoordinate);
	const SurgSim::Math::Vector& previousPosition = femElement->computeCartesianCoordinate(*currentState,
		m_position.barycentricCoordinate);

	return previousPosition + time * (currentPosition - previousPosition);
}

bool Fem3DRepresentationLocalization::isValidRepresentation(std::shared_ptr<Representation> representation)
{
	auto femRepresentation = std::dynamic_pointer_cast<Fem3DRepresentation>(representation);

	// Allows to reset the representation to nullptr ...
	return (femRepresentation != nullptr || representation == nullptr);
}

} // namespace Physics

} // namespace SurgSim
