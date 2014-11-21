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

#include "SurgSim/Particles/RandomBoxPointGenerator.h"

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Math/BoxShape.h"

namespace SurgSim
{
namespace Particles
{
using SurgSim::Math::Vector3d;

RandomBoxPointGenerator::~RandomBoxPointGenerator()
{
}

Vector3d RandomBoxPointGenerator::pointInShape(std::shared_ptr<SurgSim::Math::Shape> shape)
{
	auto box = std::static_pointer_cast<SurgSim::Math::BoxShape>(shape);
	auto halfSize = box->getSize() * 0.5;

	Vector3d random = Vector3d::NullaryExpr([&](int){return m_openOneOneDistribution(m_generator);});

	return random.array() * halfSize.array();
}

Vector3d RandomBoxPointGenerator::pointOnShape(std::shared_ptr<SurgSim::Math::Shape> shape)
{
	auto box = std::static_pointer_cast<SurgSim::Math::BoxShape>(shape);
	auto halfSize = box->getSize() * 0.5;

	std::uniform_int_distribution<int> axisDirectionSelector(0, 2); // 0: X-Axis, 1: Y-Axis, 2: Z-Axis.
	std::uniform_int_distribution<int> valueSelector(0, 1); // 0: negative size value, 1: positive size value.

	Vector3d result;
	// Choose one axis to be fixed.
	int axis = axisDirectionSelector(m_generator);
	result[axis] = valueSelector(m_generator) == 0 ? -halfSize[axis] : halfSize[axis];

	// Then generate coordinates for the other two axes.
	for (size_t t = 0; t < 2; ++t)
	{
		axis = (++axis) % 3;
		result[axis] = m_closedOneOneDistribution(m_generator) * halfSize[axis];
	}

	return result;
}

}; // namespace Particles
}; // namespace SurgSim
