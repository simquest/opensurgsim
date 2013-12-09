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

std::array<std::pair<double, double>, 1> gaussQuadrature1Point =
{{
	std::make_pair(0.0, 2.0)
}};

std::array<std::pair<double, double>, 2> gaussQuadrature2Points =
{{
	std::make_pair( 1.0/sqrt(3.0), 1.0),
	std::make_pair(-1.0/sqrt(3.0), 1.0)
}};

std::array<std::pair<double, double>, 3> gaussQuadrature3Points =
{{
	std::make_pair(           0.0, 8.0/9.0),
	std::make_pair( sqrt(3.0/5.0), 5.0/9.0),
	std::make_pair(-sqrt(3.0/5.0), 5.0/9.0)
}};

std::array<std::pair<double, double>, 4> gaussQuadrature4Points =
{{
	std::make_pair( sqrt((3.0 - 2.0*sqrt(6.0/5.0)) / 7.0), (18.0 + sqrt(30.0)) / 36.0),
	std::make_pair(-sqrt((3.0 - 2.0*sqrt(6.0/5.0)) / 7.0), (18.0 + sqrt(30.0)) / 36.0),
	std::make_pair( sqrt((3.0 + 2.0*sqrt(6.0/5.0)) / 7.0), (18.0 - sqrt(30.0)) / 36.0),
	std::make_pair(-sqrt((3.0 + 2.0*sqrt(6.0/5.0)) / 7.0), (18.0 - sqrt(30.0)) / 36.0)
}};

std::array<std::pair<double, double>, 5> gaussQuadrature5Points =
{{
	std::make_pair( 0.0, 128.0 / 225.0),
	std::make_pair( sqrt(5.0 - 2.0*sqrt(10.0/7.0)) / 3.0, (322.0 + 13.0 * sqrt(70.0)) / 900.0),
	std::make_pair(-sqrt(5.0 - 2.0*sqrt(10.0/7.0)) / 3.0, (322.0 + 13.0 * sqrt(70.0)) / 900.0),
	std::make_pair( sqrt(5.0 + 2.0*sqrt(10.0/7.0)) / 3.0, (322.0 - 13.0 * sqrt(70.0)) / 900.0),
	std::make_pair(-sqrt(5.0 + 2.0*sqrt(10.0/7.0)) / 3.0, (322.0 - 13.0 * sqrt(70.0)) / 900.0)
}};

};  // namespace Math
};  // namespace SurgSim
