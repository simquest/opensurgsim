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

#include "SurgSim/Particles/PointGenerator.h"

#include <boost/math/special_functions/next.hpp> // For boost::math::float_next()

namespace SurgSim
{

namespace Particles
{
PointGenerator::PointGenerator() :
	m_openOneOneDistribution(boost::math::float_next(-1.0), 1.0),
	m_closedOneOneDistribution(-1.0, boost::math::float_next(1.0)),
	m_closedZeroOneDistribution(0.0, boost::math::float_next(1.0)),
	m_closedZeroOpenOneDistribution(0.0, 1.0)
{
	std::random_device randomDevice;
	m_generator.seed(randomDevice());
}

PointGenerator::~PointGenerator()
{
}

}; // namespace Particles
}; // namespace SurgSim
