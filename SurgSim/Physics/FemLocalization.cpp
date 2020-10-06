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

#include "SurgSim/Physics/FemLocalization.h"

#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/FemRepresentation.h"
#include "SurgSim/Physics/FemElement.h"

namespace SurgSim
{

namespace Physics
{

FemLocalization::FemLocalization(
	std::shared_ptr<Representation> representation,
	const SurgSim::DataStructures::IndexedLocalCoordinate& localPosition) :
	Localization()
{
	setRepresentation(representation);
	setLocalPosition(localPosition);
}

FemLocalization::~FemLocalization()
{
}

void FemLocalization::setLocalPosition(
	const SurgSim::DataStructures::IndexedLocalCoordinate& localPosition)
{
	auto femRepresentation = std::static_pointer_cast<FemRepresentation>(getRepresentation());

	SURGSIM_ASSERT(femRepresentation != nullptr) << "FemRepresentation is null, it was probably not" <<
		" initialized";

	SURGSIM_ASSERT(femRepresentation->isValidCoordinate(localPosition))
		<< "IndexedLocalCoordinate is invalid for Representation " << getRepresentation()->getName();

	m_position = localPosition;
}

const SurgSim::DataStructures::IndexedLocalCoordinate& FemLocalization::getLocalPosition() const
{
	return m_position;
}

SurgSim::Math::Vector3d FemLocalization::doCalculatePosition(double time) const
{
	using SurgSim::Math::Vector3d;

	auto femRepresentation = std::static_pointer_cast<FemRepresentation>(getRepresentation());

	SURGSIM_ASSERT(femRepresentation != nullptr) << "FemRepresentation is null, it was probably not" <<
		" initialized";

	std::shared_ptr<FemElement> femElement = femRepresentation->getFemElement(m_position.index);
	Vector3d previousPosition = femElement->computeCartesianCoordinate(*femRepresentation->getPreviousState(),
		m_position.coordinate);

	if (time <= std::numeric_limits<double>::epsilon())
	{
		return previousPosition;
	}

	Vector3d currentPosition = femElement->computeCartesianCoordinate(*femRepresentation->getCurrentState(),
		m_position.coordinate);
	if (time >= 1.0 - std::numeric_limits<double>::epsilon())
	{
		return currentPosition;
	}

	return Math::interpolate(previousPosition, currentPosition, time);
}

SurgSim::Math::Vector3d FemLocalization::doCalculateVelocity(double time) const
{
	using SurgSim::Math::Vector3d;

	auto femRepresentation = std::static_pointer_cast<FemRepresentation>(getRepresentation());

	SURGSIM_ASSERT(femRepresentation != nullptr) << "FemRepresentation is null, it was probably not" <<
		" initialized";

	const SurgSim::Math::Vector& naturalCoordinate = m_position.coordinate;

	std::shared_ptr<FemElement> femElement = femRepresentation->getFemElement(m_position.index);
	SURGSIM_ASSERT(femElement->isValidCoordinate(naturalCoordinate)) <<
		"Invalid naturalCoordinate (" << naturalCoordinate.transpose() << ")";
	const Math::Vector& currentVelocities = femRepresentation->getCurrentState()->getVelocities();
	const Math::Vector& previousVelocities = femRepresentation->getPreviousState()->getVelocities();

	auto& nodeIds = femElement->getNodeIds();
	Vector3d previousVelocity = Vector3d::Zero();
	for (Eigen::Index i = 0; i < naturalCoordinate.size(); ++i)
	{
		previousVelocity += naturalCoordinate(i) * Math::getSubVector(previousVelocities, nodeIds[i], 6).segment<3>(0);
	}
	if (time <= std::numeric_limits<double>::epsilon())
	{
		return previousVelocity;
	}

	Vector3d currentVelocity = Vector3d::Zero();
	for (Eigen::Index i = 0; i < naturalCoordinate.size(); ++i)
	{
		currentVelocity += naturalCoordinate(i) * Math::getSubVector(currentVelocities, nodeIds[i], 6).segment<3>(0);
	}
	if (time >= 1.0 - std::numeric_limits<double>::epsilon())
	{
		return currentVelocity;
	}

	return Math::interpolate(previousVelocity, currentVelocity, time);
}

} // namespace Physics

} // namespace SurgSim
