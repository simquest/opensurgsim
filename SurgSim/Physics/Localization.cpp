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

#include "SurgSim/Physics/Localization.h"

#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Physics/Representation.h"

namespace SurgSim
{
namespace Physics
{

Localization::Localization()
{
}

Localization::Localization(std::shared_ptr<Representation> representation) : m_representation(representation)
{
}

Localization::~Localization()
{
}

void Localization::setRepresentation(std::shared_ptr<Representation> representation)
{
	if (isValidRepresentation(representation))
	{
		m_representation = representation;
	}
	else
	{
		SURGSIM_FAILURE() << "Unexpected representation type" << std::endl;
	}
}

std::shared_ptr<Representation> Localization::getRepresentation() const
{
	return m_representation;
}

Math::Vector3d Localization::calculatePosition(double time)
{
	return doCalculatePosition(time);
}

Math::Vector3d Localization::calculateVelocity(double time)
{
	return doCalculateVelocity(time);
}

bool Localization::isValidRepresentation(std::shared_ptr<Representation> representation)
{
	// Localization base class does not care about the type
	return true;
}

Math::RigidTransform3d Localization::getElementPose()
{
	SURGSIM_FAILURE() << "Localization::getElementPose() is not implemented for " <<
		getRepresentation()->getFullName();
	return Math::RigidTransform3d();
}

bool Localization::moveClosestTo(const Math::Vector3d& point, bool *hasReachedEnd)
{
	SURGSIM_FAILURE() << "Localization::moveClosestTo() is not implemented for " << getRepresentation()->getFullName();
	return false;
}

}
}
