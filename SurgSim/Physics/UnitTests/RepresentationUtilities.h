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

#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>

namespace SurgSim
{
namespace Physics
{
class CollisionRepresentation;
class Representation;

std::shared_ptr<CollisionRepresentation> makeSphereRepresentation(
	std::shared_ptr<SurgSim::Physics::Representation> representation = nullptr,
	const double& radius = 1.0,
	const SurgSim::Math::Quaterniond& rotation = SurgSim::Math::Quaterniond::Identity(),
	const SurgSim::Math::Vector3d& position = SurgSim::Math::Vector3d::Zero());

std::shared_ptr<CollisionRepresentation> makeDoubleSidedPlaneRepresentation(
	std::shared_ptr<SurgSim::Physics::Representation> representation = nullptr,
	const SurgSim::Math::Quaterniond& rotation = SurgSim::Math::Quaterniond::Identity(),
	const SurgSim::Math::Vector3d& position = SurgSim::Math::Vector3d::Zero());

std::shared_ptr<CollisionRepresentation> makePlaneRepresentation(
	std::shared_ptr<SurgSim::Physics::Representation> representation = nullptr,
	const SurgSim::Math::Quaterniond& rotation = SurgSim::Math::Quaterniond::Identity(),
	const SurgSim::Math::Vector3d& position = SurgSim::Math::Vector3d::Zero());

std::shared_ptr<CollisionRepresentation> makeCapsuleRepresentation(
	std::shared_ptr<SurgSim::Physics::Representation> representation = nullptr,
	const double& length = 1.0,
	const double& radius = 1.0,
	const SurgSim::Math::Quaterniond& rotation = SurgSim::Math::Quaterniond::Identity(),
	const SurgSim::Math::Vector3d& position = SurgSim::Math::Vector3d::Zero());
}; // Physics
}; // SurgSim

#endif