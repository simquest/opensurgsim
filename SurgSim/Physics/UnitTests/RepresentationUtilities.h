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

#ifndef SURGSIM_PHYSICS_UNITTESTS_REPRESENTATIONUTILITIES_H
#define SURGSIM_PHYSICS_UNITTESTS_REPRESENTATIONUTILITIES_H

#include <memory>

#include<SurgSim/Physics/CollisionRepresentation.h>

#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>

namespace SurgSim
{
namespace Physics
{

std::shared_ptr<CollisionRepresentation> makeSphereRepresentation(
	const double& radius,
	const SurgSim::Math::Quaterniond& rotation = SurgSim::Math::Quaterniond::Identity(),
	const SurgSim::Math::Vector3d& position = SurgSim::Math::Vector3d::Zero());

std::shared_ptr<CollisionRepresentation> makeDoubleSidedPlaneRepresentation(
	const SurgSim::Math::Quaterniond& rotation = SurgSim::Math::Quaterniond::Identity(),
	const SurgSim::Math::Vector3d& position = SurgSim::Math::Vector3d::Zero());

std::shared_ptr<CollisionRepresentation> makePlaneRepresentation(
	const SurgSim::Math::Quaterniond& rotation = SurgSim::Math::Quaterniond::Identity(),
	const SurgSim::Math::Vector3d& position = SurgSim::Math::Vector3d::Zero());

}; // Physics
}; // SurgSim

#endif