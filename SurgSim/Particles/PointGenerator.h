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

#ifndef SURGSIM_PARTICLES_POINTGENERATOR_H
#define SURGSIM_PARTICLES_POINTGENERATOR_H

#include <memory>
#include <random>

#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Math
{
class Shape;
}

namespace Particles
{

/// PointGenerator is used to generate points inside or on the surface of a given shape.
/// Derived classes need to implement pointInShape() and pointOnShape().
class PointGenerator
{
public:
	/// Constructor
	PointGenerator();

	/// Destructor
	virtual ~PointGenerator();

	/// Generates one point inside the given shape.
	/// \param shape The shape inside which a point will be generated.
	/// \return A point inside the shape, shape is assumed to be located at the origin.
	virtual SurgSim::Math::Vector3d pointInShape(std::shared_ptr<SurgSim::Math::Shape> shape) = 0;

	/// Generates one point on the surface of the given shape.
	/// \param shape The shape on which a point will be generated.
	/// \return A point on the surface of the shape, shape is assumed to be located at the origin.
	virtual SurgSim::Math::Vector3d pointOnShape(std::shared_ptr<SurgSim::Math::Shape> shape) = 0;

protected:
	///@{
	/// Random number generator and some predefined distributions to be used by different shape point generators.
	std::mt19937 m_generator;

	std::uniform_real_distribution<double> m_openOneOneDistribution; // <-- (-1.0, 1.0)
	std::uniform_real_distribution<double> m_closedOneOneDistribution; // <-- [-1.0, 1.0]
	std::uniform_real_distribution<double> m_closedZeroOneDistribution; // <-- [0.0, 1.0]
	std::uniform_real_distribution<double> m_closedZeroOpenOneDistribution; // <-- [0.0, 1.0)
	///@}
};

}; // namespace Particles
}; // namespace SurgSim

#endif // SURGSIM_PARTICLES_POINTGENERATOR_H
