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
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"

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
	m_nodeID = nodeID;
}

const size_t& MassSpringLocalization::getLocalNode() const
{
	return m_nodeID;
}

SurgSim::Math::Vector3d MassSpringLocalization::doCalculatePosition(double time) const
{
	std::shared_ptr<MassSpringRepresentation> massSpringRepresentation =
		std::static_pointer_cast<MassSpringRepresentation>(getRepresentation());

	SURGSIM_ASSERT(massSpringRepresentation != nullptr) << "MassSpringRepresentation is null, it was probably not" <<
		" initialized";

	if (time <= std::numeric_limits<double>::epsilon())
	{
		return massSpringRepresentation->getPreviousState()->getPosition(m_nodeID);
	}
	else if (time >= 1.0 - std::numeric_limits<double>::epsilon())
	{
		return massSpringRepresentation->getCurrentState()->getPosition(m_nodeID);
	}

	const SurgSim::Math::Vector3d& currentPoint  = massSpringRepresentation->getCurrentState()->getPosition(m_nodeID);
	const SurgSim::Math::Vector3d& previousPoint = massSpringRepresentation->getPreviousState()->getPosition(m_nodeID);

	return SurgSim::Math::interpolate(previousPoint, currentPoint, time);
}

SurgSim::Math::Vector3d MassSpringLocalization::doCalculateVelocity(double time) const
{
	std::shared_ptr<MassSpringRepresentation> massSpringRepresentation =
		std::static_pointer_cast<MassSpringRepresentation>(getRepresentation());

	SURGSIM_ASSERT(massSpringRepresentation != nullptr) << "MassSpringRepresentation is null, it was probably not" <<
		" initialized";

	if (time <= std::numeric_limits<double>::epsilon())
	{
		return massSpringRepresentation->getPreviousState()->getVelocity(m_nodeID);
	}
	else if (time >= 1.0 - std::numeric_limits<double>::epsilon())
	{
		return massSpringRepresentation->getCurrentState()->getVelocity(m_nodeID);
	}

	const SurgSim::Math::Vector3d& currentPoint = massSpringRepresentation->getCurrentState()->getVelocity(m_nodeID);
	const SurgSim::Math::Vector3d& previousPoint = massSpringRepresentation->getPreviousState()->getVelocity(m_nodeID);

	return SurgSim::Math::interpolate(previousPoint, currentPoint, time);
}

bool MassSpringLocalization::isValidRepresentation(std::shared_ptr<Representation> representation)
{

	std::shared_ptr<MassSpringRepresentation> massSpringRepresentation =
		std::dynamic_pointer_cast<MassSpringRepresentation>(representation);

	// Allows to reset the representation to nullptr ...
	return (massSpringRepresentation != nullptr || representation == nullptr);
}

}; // Physics
}; // SurgSim


