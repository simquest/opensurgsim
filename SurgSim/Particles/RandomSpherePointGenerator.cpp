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

#include "SurgSim/Particles/RandomSpherePointGenerator.h"

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Math/SphereShape.h"

namespace SurgSim
{
namespace Particles
{
using SurgSim::Math::Vector3d;

RandomSpherePointGenerator::~RandomSpherePointGenerator()
{
}

Vector3d RandomSpherePointGenerator::pointInShape(std::shared_ptr<SurgSim::Math::Shape> shape)
{
	auto sphere = std::static_pointer_cast<SurgSim::Math::SphereShape>(shape);

	// A point on the sphere, use it as direction and then multiple it by a number between [0,1) to move it inside.
	Vector3d result = pointOnShape(shape) * m_closedZeroOpenOneDistribution(m_generator);

	return result;
}

Vector3d RandomSpherePointGenerator::pointOnShape(std::shared_ptr<SurgSim::Math::Shape> shape)
{
	auto sphere = std::static_pointer_cast<SurgSim::Math::SphereShape>(shape);
	double radius = sphere->getRadius();

	// Spherical coordinate system can not produce an uniformly distributed points on the surface of the sphere.
	// ref: http://mathworld.wolfram.com/SpherePointPicking.html

	// Implementation was based on http://www.cs.cmu.edu/~mws/rpos.html
	double z = 0.0;
	double phi = 0.0;
	double theta = 0.0;
	double cosineTheta = 0.0;

	Vector3d result = Vector3d::Zero();
	// If the origin (0.0, 0.0, 0.0) is produced, regenerate.
	while (result.isZero())
	{
		z = m_closedOneOneDistribution(m_generator) * radius;
		phi = m_closedZeroOneDistribution(m_generator) * 2.0 * M_PI;
		theta = std::asin(z / radius);

		cosineTheta = std::cos(theta);

		result.x() = radius * cosineTheta * std::cos(phi);
		result.y() = radius * cosineTheta * std::sin(phi);
		result.z() = z;
	}

	result.normalize();
	result *= radius;

	return result;
}

}; // namespace Particles
}; // namespace SurgSim
