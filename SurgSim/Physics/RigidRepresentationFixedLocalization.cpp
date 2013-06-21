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

#include <SurgSim/Physics/RigidRepresentationFixedLocalization.h>

namespace SurgSim
{
namespace Physics
{
RigidRepresentationFixedLocalization::RigidRepresentationFixedLocalization() :
	Localization()
{

}

RigidRepresentationFixedLocalization::RigidRepresentationFixedLocalization(std::shared_ptr<Representation> representation) :
	Localization(representation)
{
	std::shared_ptr<RigidRepresentationBase> rigidRepresentation = 
		std::dynamic_pointer_cast<RigidRepresentationBase>(representation);
	SURGSIM_ASSERT(rigidRepresentation != nullptr) << "Unexpected representation type" << std::endl;
}

RigidRepresentationFixedLocalization::~RigidRepresentationFixedLocalization()
{

}

void RigidRepresentationFixedLocalization::setLocalPosition(const SurgSim::Math::Vector3d& p)
{
	m_position = p;
}

const SurgSim::Math::Vector3d& RigidRepresentationFixedLocalization::getLocalPosition() const
{
	return m_position;
}

bool RigidRepresentationFixedLocalization::isEqual(const Localization& localization) const
{
	const RigidRepresentationFixedLocalization& fixedLoc = 
		static_cast<const RigidRepresentationFixedLocalization&>(localization);
	return m_position == fixedLoc.m_position;
}

SurgSim::Math::Vector3d RigidRepresentationFixedLocalization::doCalculatePosition(double time)
{
	std::shared_ptr<RigidRepresentationBase> rigidRepresentation = 
		std::static_pointer_cast<RigidRepresentationBase>(getRepresentation());

	if (time == 0.0)
	{
		return rigidRepresentation->getPreviousState().getPose() * m_position;
	}
	else if (time == 1.0)
	{
		return rigidRepresentation->getCurrentState().getPose() * m_position;
	}
	else if (rigidRepresentation->getCurrentState().getPose().isApprox(rigidRepresentation->getPreviousState().getPose()))
	{
		return rigidRepresentation->getCurrentState().getPose() * m_position;
	}

	const SurgSim::Math::RigidTransform3d& currentPose  = rigidRepresentation->getCurrentState().getPose();
	const SurgSim::Math::RigidTransform3d& previousPose = rigidRepresentation->getPreviousState().getPose();
	SurgSim::Math::RigidTransform3d pose = SurgSim::Math::interpolate(previousPose, currentPose, time);

	return pose * m_position;
}


}; // Physics
}; // SurgSim
