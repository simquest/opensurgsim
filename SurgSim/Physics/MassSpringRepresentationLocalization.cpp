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

#include "SurgSim/Physics/MassSpringRepresentationLocalization.h"

namespace SurgSim
{
namespace Physics
{
MassSpringRepresentationLocalization::MassSpringRepresentationLocalization()
{

}

MassSpringRepresentationLocalization::MassSpringRepresentationLocalization(
	std::shared_ptr<Representation> representation) :
	Localization()
{
	setRepresentation(representation);
}

MassSpringRepresentationLocalization::~MassSpringRepresentationLocalization()
{

}

void MassSpringRepresentationLocalization::setLocalNode(unsigned int nodeID)
{
	m_nodeID = nodeID;
}

const size_t& MassSpringRepresentationLocalization::getLocalNode() const
{
	return m_nodeID;
}

SurgSim::Math::Vector3d MassSpringRepresentationLocalization::doCalculatePosition(double time)
{
	std::shared_ptr<MassSpringRepresentation> massSpringRepresentation =
		std::static_pointer_cast<MassSpringRepresentation>(getRepresentation());

	SURGSIM_ASSERT(massSpringRepresentation != nullptr) << "MassSpringRepresentation is null, it was probably not" <<
		" initialized";
	SURGSIM_ASSERT((0.0 <= time) && (time <= 1.0)) << "Time must be between 0.0 and 1.0 inclusive";

	if (time == 0.0)
	{
		return massSpringRepresentation->getPreviousState()->getPosition(m_nodeID);
	}
	else if (time == 1.0)
	{
		return massSpringRepresentation->getCurrentState()->getPosition(m_nodeID);
	}
	else if (massSpringRepresentation->getCurrentState()->getPosition(m_nodeID).
		isApprox(massSpringRepresentation->getPreviousState()->getPosition(m_nodeID)))
	{
		return massSpringRepresentation->getCurrentState()->getPosition(m_nodeID);
	}

	const SurgSim::Math::Vector3d& currentPose  = massSpringRepresentation->getCurrentState()->getPosition(m_nodeID);
	const SurgSim::Math::Vector3d& previousPose = massSpringRepresentation->getPreviousState()->getPosition(m_nodeID);

	return previousPose + time * (currentPose - previousPose);
}

bool MassSpringRepresentationLocalization::isValidRepresentation(std::shared_ptr<Representation> representation)
{

	std::shared_ptr<MassSpringRepresentation> massSpringRepresentation =
		std::dynamic_pointer_cast<MassSpringRepresentation>(representation);

	// Allows to reset the representation to nullptr ...
	return (massSpringRepresentation != nullptr || representation == nullptr);
}

}; // Physics
}; // SurgSim


