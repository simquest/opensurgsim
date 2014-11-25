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

#include "SurgSim/Particles/DefaultPointGenerator.h"

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{
namespace Particles
{
using SurgSim::Math::Vector3d;

DefaultPointGenerator::~DefaultPointGenerator()
{
}

Vector3d DefaultPointGenerator::pointInShape(std::shared_ptr<SurgSim::Math::Shape> shape)
{
	SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << 
		"PointGenerator does not support generate point in shape: "<< shape->getType() << " yet.";

	return Vector3d::Zero();
}

Vector3d DefaultPointGenerator::pointOnShape(std::shared_ptr<SurgSim::Math::Shape> shape)
{
	SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << 
		"PointGenerator does not support generate point on the surface of shape: "<< shape->getType() << " yet.";

	return Vector3d::Zero();
}

}; // namespace Particles
}; // namespace SurgSim
