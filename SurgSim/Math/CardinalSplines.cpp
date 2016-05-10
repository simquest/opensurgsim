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

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Math/CardinalSplines.h"

namespace SurgSim
{
namespace Math
{
namespace CardinalSplines
{

void extendControlPoints(const SurgSim::DataStructures::VerticesPlain& points,
						 std::vector<SurgSim::Math::Vector3d>* result)
{
	SURGSIM_ASSERT(points.getNumVertices() >= 2) << "Cannot extend control points less than 2.";
	SURGSIM_ASSERT(result != nullptr) << "'result' can not be nullptr.";
	result->clear();
	result->reserve(points.getNumVertices() + 2);

	// Interpolate the 1st point (ghost) as the symmetric of P1 from P0: P-1 = P0 - (P1 - P0)
	result->push_back(2.0 * points.getVertexPosition(0) - points.getVertexPosition(1));
	for (size_t i = 0; i < points.getNumVertices(); ++i)
	{
		result->push_back(points.getVertexPosition(i));
	}
	// Interpolate the last point (ghost) as the symmetric of Pn-1 from Pn: Pn+1 = Pn - (Pn-1 - Pn)
	result->push_back(2.0 * points.getVertexPosition(points.getNumVertices() - 1) -
					  points.getVertexPosition(points.getNumVertices() - 2));
}

void interpolate(size_t subdivisions,
				 const std::vector<SurgSim::Math::Vector3d>& controlPoints,
				 std::vector<SurgSim::Math::Vector3d>* points,
				 double tau)
{
	SURGSIM_ASSERT(subdivisions >= 1) << "'subdivision' must be at least 1.";
	SURGSIM_ASSERT(controlPoints.size() >= 4) << "Cannot apply Cardinal Splines interpolation with less than 4 points";
	SURGSIM_ASSERT(points != nullptr) << "'points' is nullptr";
	SURGSIM_ASSERT(0 <= tau && tau <= 1) << "Tension parameter 'tau' must be in the range [0,1].";

	size_t numPoints = controlPoints.size();
	double stepsize = 1.0 / static_cast<double>(subdivisions);
	for (size_t pointIndex = 0; pointIndex < numPoints - 3; ++pointIndex)
	{
		std::array<SurgSim::Math::Vector3d, 4> p =
		{
			controlPoints[pointIndex],
			controlPoints[pointIndex + 1],
			controlPoints[pointIndex + 2],
			controlPoints[pointIndex + 3]
		};

		double abscissa = 0.0;
		/*
		Note that 'controlPoints' are NOT included in the final result, 'points'.
		However, when 'abscissa' is 0.0, the interpolated point will happen to have the same value with one of the
		control points and thus being included in the result.
		*/
		for (size_t i = 0; i < subdivisions; ++i)
		{
			double abcissaSquared = abscissa * abscissa;
			double abcissaCubed = abcissaSquared * abscissa;

			SurgSim::Math::Vector3d result =
				p[1] +
				abscissa * (tau * (p[2] - p[0])) +
				abcissaSquared * (2.0 * tau * p[0] + (tau - 3.0) * p[1] + (3.0 - 2.0 * tau) * p[2] - tau * p[3]) +
				abcissaCubed * (-tau * p[0] + (2.0 - tau) * p[1] + (tau - 2.0) * p[2] + tau * p[3]);

			points->push_back(std::move(result));

			abscissa += stepsize;
		}
	}
}

}; // namespace CardinalSplines
}; // namespace Math
}; // namespace SurgSim
