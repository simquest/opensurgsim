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

	Vector3d result;
	result.x() = m_openOneOneDistribution(m_generator) * halfSize.x();
	result.y() = m_openOneOneDistribution(m_generator) * halfSize.y();
	result.z() = m_openOneOneDistribution(m_generator) * halfSize.z();

	return result;
}

Vector3d RandomBoxPointGenerator::pointOnShape(std::shared_ptr<SurgSim::Math::Shape> shape)
{
	auto box = std::static_pointer_cast<SurgSim::Math::BoxShape>(shape);
	auto halfSize = box->getSize() * 0.5;

	Vector3d result;

	// First choose one out of (x,y,z) to be fixed, and then randomly generate the other two coordinates.
	std::uniform_int_distribution<int> axisDirectionSelector(0, 2); // 0: X-Axis, 1: Y-Axis, 2: Z-Axis.
	std::uniform_int_distribution<int> valueSelector(0, 1); // 0: negative size value, 1: positive size value.
	switch (axisDirectionSelector(m_generator))
	{
	case 0:
		result.x() = (valueSelector(m_generator) == 0 ? -halfSize.x() : halfSize.x());
		result.y() = m_closedOneOneDistribution(m_generator) * halfSize.y();
		result.z() = m_closedOneOneDistribution(m_generator) * halfSize.z();;
		break;
	case 1:
		result.x() = m_closedOneOneDistribution(m_generator) * halfSize.x();;
		result.y() = (valueSelector(m_generator) == 0 ? -halfSize.y() : halfSize.y());
		result.z() = m_closedOneOneDistribution(m_generator) * halfSize.z();;
		break;
	case 2:
		result.x() = m_closedOneOneDistribution(m_generator) * halfSize.x();;
		result.y() = m_closedOneOneDistribution(m_generator) * halfSize.y();;
		result.z() = (valueSelector(m_generator) == 0 ? -halfSize.z() : halfSize.z());
		break;
	default:
		SURGSIM_FAILURE() << "Failed to generate a point";
		break;
	}

	return result;
}

}; // namespace Particles
}; // namespace SurgSim
