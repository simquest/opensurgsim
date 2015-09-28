// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PARTICLES_RANDOMMESHPOINTGENERATOR_H
#define SURGSIM_PARTICLES_RANDOMMESHPOINTGENERATOR_H

#include "SurgSim/Particles/PointGenerator.h"


namespace SurgSim
{

namespace Math
{
class Shape;
}

namespace Particles
{

/// Class to generate points on the surface of a mesh
/// \note Each triangle in the mesh has equal weight in the distribution of
/// points. As a result, areas with higher triangle density have a higher
/// likely hood of generating points than areas with a lower triangle density.
class RandomMeshPointGenerator: public PointGenerator
{
public:
	Math::Vector3d pointInShape(std::shared_ptr<Math::Shape> shape) override;

	Math::Vector3d pointOnShape(std::shared_ptr<Math::Shape> shape) override;
};

}; // namespace Particles
}; // namespace SurgSim


#endif // SURGSIM_PARTICLES_RANDOMMESHPOINTGENERATOR_H
