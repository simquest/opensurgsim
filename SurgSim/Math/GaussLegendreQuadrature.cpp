// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

#include <math.h>

#include "SurgSim/Math/GaussLegendreQuadrature.h"

namespace SurgSim
{
namespace Math
{

std::array<gaussQuadraturePoint, 1> gaussQuadrature1Point =
{{
	gaussQuadraturePoint(0.0, 2.0)
}};

std::array<gaussQuadraturePoint, 2> gaussQuadrature2Points =
{
	gaussQuadraturePoint( 1.0 / sqrt(3.0), 1.0),
	gaussQuadraturePoint(-1.0 / sqrt(3.0), 1.0)
};

std::array<gaussQuadraturePoint, 3> gaussQuadrature3Points =
{{
	gaussQuadraturePoint(             0.0, 8.0 / 9.0),
	gaussQuadraturePoint( sqrt(3.0 / 5.0), 5.0 / 9.0),
	gaussQuadraturePoint(-sqrt(3.0 / 5.0), 5.0 / 9.0)
}};

std::array<gaussQuadraturePoint, 4> gaussQuadrature4Points =
{{
	gaussQuadraturePoint( sqrt((3.0 - 2.0 * sqrt(6.0 / 5.0)) / 7.0), (18.0 + sqrt(30.0)) / 36.0),
	gaussQuadraturePoint(-sqrt((3.0 - 2.0 * sqrt(6.0 / 5.0)) / 7.0), (18.0 + sqrt(30.0)) / 36.0),
	gaussQuadraturePoint( sqrt((3.0 + 2.0 * sqrt(6.0 / 5.0)) / 7.0), (18.0 - sqrt(30.0)) / 36.0),
	gaussQuadraturePoint(-sqrt((3.0 + 2.0 * sqrt(6.0 / 5.0)) / 7.0), (18.0 - sqrt(30.0)) / 36.0)
}};

std::array<gaussQuadraturePoint, 5> gaussQuadrature5Points =
{{
	gaussQuadraturePoint( 0.0, 128.0 / 225.0),
	gaussQuadraturePoint( sqrt(5.0 - 2.0 * sqrt(10.0 / 7.0)) / 3.0, (322.0 + 13.0 * sqrt(70.0)) / 900.0),
	gaussQuadraturePoint(-sqrt(5.0 - 2.0 * sqrt(10.0 / 7.0)) / 3.0, (322.0 + 13.0 * sqrt(70.0)) / 900.0),
	gaussQuadraturePoint( sqrt(5.0 + 2.0 * sqrt(10.0 / 7.0)) / 3.0, (322.0 - 13.0 * sqrt(70.0)) / 900.0),
	gaussQuadraturePoint(-sqrt(5.0 + 2.0 * sqrt(10.0 / 7.0)) / 3.0, (322.0 - 13.0 * sqrt(70.0)) / 900.0)
}};

};  // namespace Math
};  // namespace SurgSim
