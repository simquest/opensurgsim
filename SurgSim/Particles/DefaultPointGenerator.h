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

#ifndef SURGSIM_PARTICLES_DEFAULTPOINTGENERATOR_H
#define SURGSIM_PARTICLES_DEFAULTPOINTGENERATOR_H

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

/// DefaultPointGenerator, methods of this class will always return (0.0, 0.0, 0.0) and output a severe logging message.
/// They are served more like a place holder (concrete implementation) to be used in ShapesPointGenerator.
/// One should develop an actual XXXPointGenerator to produce points in/on the shape.
class DefaultPointGenerator: public PointGenerator
{
public:
	/// Destructor
	virtual ~DefaultPointGenerator();

	SurgSim::Math::Vector3d pointInShape(std::shared_ptr<SurgSim::Math::Shape> shape) override;
	SurgSim::Math::Vector3d pointOnShape(std::shared_ptr<SurgSim::Math::Shape> shape) override;
};

}; // namespace Particles
}; // namespace SurgSim

#endif // SURGSIM_PARTICLES_DEFAULTPOINTGENERATOR_H
