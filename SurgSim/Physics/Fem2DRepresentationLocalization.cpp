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

#include "SurgSim/Physics/Fem2DRepresentationLocalization.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"
#include "SurgSim/Physics/FemElement.h"

namespace SurgSim
{

namespace Physics
{

Fem2DRepresentationLocalization::Fem2DRepresentationLocalization()
{

}

Fem2DRepresentationLocalization::Fem2DRepresentationLocalization(std::shared_ptr<Representation> representation) :
	Localization()
{
	setRepresentation(representation);
}

Fem2DRepresentationLocalization::~Fem2DRepresentationLocalization()
{

}

void Fem2DRepresentationLocalization::setLocalPosition(const SurgSim::DataStructures::IndexedLocalCoordinate& p)
{
	auto femRepresentation = std::static_pointer_cast<Fem2DRepresentation>(getRepresentation());

	SURGSIM_ASSERT(femRepresentation != nullptr) << "FemRepresentation is null, it was probably not" <<
		" initialized";

	SURGSIM_ASSERT(femRepresentation->isValidCoordinate(p))
		<< "IndexedLocalCoordinate is invalid for Representation " << getRepresentation()->getName();

	m_position = p;
}

const SurgSim::DataStructures::IndexedLocalCoordinate& Fem2DRepresentationLocalization::getLocalPosition() const
{
	return m_position;
}

SurgSim::Math::Vector3d Fem2DRepresentationLocalization::doCalculatePosition(double time)
{
	auto femRepresentation = std::static_pointer_cast<Fem2DRepresentation>(getRepresentation());

	SURGSIM_ASSERT(femRepresentation != nullptr) << "FemRepresentation is null, it was probably not" <<
		" initialized";

	std::shared_ptr<FemElement> femElement = femRepresentation->getFemElement(m_position.index);
	const std::shared_ptr<SurgSim::Math::OdeState> previousState = femRepresentation->getPreviousState();
	const std::shared_ptr<SurgSim::Math::OdeState> currentState = femRepresentation->getCurrentState();

	if (time == 0.0)
	{
		return femElement->computeCartesianCoordinate(*previousState, m_position.coordinate);
	}
	else if (time == 1.0)
	{
		return femElement->computeCartesianCoordinate(*currentState, m_position.coordinate);
	}

	const SurgSim::Math::Vector& currentPosition = femElement->computeCartesianCoordinate(*previousState,
		m_position.coordinate);
	const SurgSim::Math::Vector& previousPosition = femElement->computeCartesianCoordinate(*currentState,
		m_position.coordinate);

	return previousPosition + time * (currentPosition - previousPosition);
}

bool Fem2DRepresentationLocalization::isValidRepresentation(std::shared_ptr<Representation> representation)
{
	auto femRepresentation = std::dynamic_pointer_cast<Fem2DRepresentation>(representation);

	// Allows to reset the representation to nullptr ...
	return (femRepresentation != nullptr || representation == nullptr);
}

} // namespace Physics

} // namespace SurgSim
