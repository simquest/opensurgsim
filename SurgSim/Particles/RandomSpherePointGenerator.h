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

#ifndef SURGSIM_PARTICLES_RANDOMSPHEREPOINTGENERATOR_H
#define SURGSIM_PARTICLES_RANDOMSPHEREPOINTGENERATOR_H

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/PointGenerator.h"

namespace SurgSim
{

namespace Math
{
class Shape;
}

namespace Particles
{

// Generates points from a box
class RandomSpherePointGenerator: public PointGenerator
{
public:
	/// Destructor
	virtual ~RandomSpherePointGenerator();

	virtual SurgSim::Math::Vector3d pointInShape(std::shared_ptr<SurgSim::Math::Shape> shape) override;
	virtual SurgSim::Math::Vector3d pointOnShape(std::shared_ptr<SurgSim::Math::Shape> shape) override;
};

}; // namespace Particles
}; // namespace SurgSim

#endif // SURGSIM_PARTICLES_RANDOMSPHEREPOINTGENERATOR_H
