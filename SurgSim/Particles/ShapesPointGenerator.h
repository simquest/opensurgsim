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

#ifndef SURGSIM_PARTICLES_SHAPESPOINTGENERATOR_H
#define SURGSIM_PARTICLES_SHAPESPOINTGENERATOR_H

#include <array>
#include <memory>

#include "SurgSim/Math/Shape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/PointGenerator.h"

namespace SurgSim
{

namespace Particles
{
/// ShapesPointGenerator is a list of PointGenerators.
/// The list gets populated when ShapesPointGeneartor is constructed.
/// Based on the shape passed, ShapesPointGenerator invokes the corresponding PointGenerator.
/// \sa PointGenerator
class ShapesPointGenerator : public PointGenerator
{
public:
	/// Constructor
	ShapesPointGenerator();

	virtual SurgSim::Math::Vector3d pointInShape(std::shared_ptr<SurgSim::Math::Shape> shape) override;
	virtual SurgSim::Math::Vector3d pointOnShape(std::shared_ptr<SurgSim::Math::Shape> shape) override;

private:
	/// List of point generators.
	/// Will be populated by constructor.
	std::array<std::unique_ptr<PointGenerator>, SurgSim::Math::SHAPE_TYPE_COUNT> m_pointGenerators;
};

}; // namespace Particles
}; // namespace SurgSim

#endif // SURGSIM_PARTICLES_SHAPESPOINTGENERATOR_H
