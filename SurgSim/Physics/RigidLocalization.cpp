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

#include "SurgSim/Physics/RigidLocalization.h"

#include "SurgSim/Physics/RigidRepresentationBase.h"

namespace SurgSim
{
namespace Physics
{
RigidLocalization::RigidLocalization() :
	Localization()
{

}

RigidLocalization::RigidLocalization(std::shared_ptr<Representation>
																		 representation) :
	Localization()
{
	setRepresentation(representation);
}

RigidLocalization::~RigidLocalization()
{

}

void RigidLocalization::setLocalPosition(const SurgSim::Math::Vector3d& p)
{
	m_position = p;
}

const SurgSim::Math::Vector3d& RigidLocalization::getLocalPosition() const
{
	return m_position;
}

SurgSim::Math::Vector3d RigidLocalization::doCalculatePosition(double time)
{
	std::shared_ptr<RigidRepresentationBase> rigidRepresentation =
		std::static_pointer_cast<RigidRepresentationBase>(getRepresentation());

	SURGSIM_ASSERT(rigidRepresentation != nullptr) << "RigidRepresentation is null, it was probably not" <<
		" initialized";

	if (time == 0.0)
	{
		return rigidRepresentation->getPreviousState().getPose() * m_position;
	}
	else if (time == 1.0)
	{
		return rigidRepresentation->getCurrentState().getPose() * m_position;
	}
	else if (rigidRepresentation->getCurrentState().getPose().
		isApprox(rigidRepresentation->getPreviousState().getPose()))
	{
		return rigidRepresentation->getCurrentState().getPose() * m_position;
	}

	const SurgSim::Math::RigidTransform3d& currentPose  = rigidRepresentation->getCurrentState().getPose();
	const SurgSim::Math::RigidTransform3d& previousPose = rigidRepresentation->getPreviousState().getPose();
	SurgSim::Math::RigidTransform3d pose = SurgSim::Math::interpolate(previousPose, currentPose, time);

	return pose * m_position;
}

SurgSim::Math::Vector3d RigidLocalization::doCalculateVelocity(double time)
{
	std::shared_ptr<RigidRepresentationBase> rigidRepresentation =
		std::static_pointer_cast<RigidRepresentationBase>(getRepresentation());

	SURGSIM_ASSERT(rigidRepresentation != nullptr) << "RigidRepresentation is null, it was probably not" <<
		" initialized";

	if (time == 0.0)
	{
		// V(P) = v + w.cross(GP)
		return rigidRepresentation->getPreviousState().getLinearVelocity() + 
			rigidRepresentation->getPreviousState().getAngularVelocity().cross(rigidRepresentation->getPreviousState().getPose().linear() * m_position);
	}
	else if (time == 1.0)
	{
		return rigidRepresentation->getCurrentState().getLinearVelocity() +
			rigidRepresentation->getCurrentState().getAngularVelocity().cross(rigidRepresentation->getCurrentState().getPose().linear() * m_position);
	}

	Math::Vector3d currentVelocity = rigidRepresentation->getCurrentState().getLinearVelocity() +
		rigidRepresentation->getCurrentState().getAngularVelocity().cross(rigidRepresentation->getCurrentState().getPose().linear() * m_position);
	Math::Vector3d previousVelocity = rigidRepresentation->getPreviousState().getLinearVelocity() +
		rigidRepresentation->getPreviousState().getAngularVelocity().cross(rigidRepresentation->getPreviousState().getPose().linear() * m_position);

	return currentVelocity * time + previousVelocity * (1.0 - time);
}

bool RigidLocalization::isValidRepresentation(std::shared_ptr<Representation> representation)
{

	std::shared_ptr<RigidRepresentationBase> rigidRepresentation =
		std::dynamic_pointer_cast<RigidRepresentationBase>(representation);

	// Allows to reset the representation to nullptr ...
	return (rigidRepresentation != nullptr || representation == nullptr);
}

}; // Physics
}; // SurgSim
