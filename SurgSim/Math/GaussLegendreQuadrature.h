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

/// \file
/// Definitions of a n-point Gaussian quadrature rule (a.k.a. Gauss-Legendre quadrature rule)
/// http://en.wikipedia.org/wiki/Gaussian_quadrature

#ifndef SURGSIM_MATH_GAUSSLEGENDREQUADRATURE_H
#define SURGSIM_MATH_GAUSSLEGENDREQUADRATURE_H

#include <array>
#include <utility>

namespace SurgSim
{
namespace Math
{

struct gaussQuadraturePoint
{
	gaussQuadraturePoint(double p, double w) : point(p), weight(w){}

	const double point;
	const double weight;
};

/// 1-point Gauss-Legendre quadrature {<x_1, w_1>}
/// \note Gauss-Legendre quadrature numerically evaluates the integral of a function \f$f\f$ with a finite sum
/// using some weights and specific points of evaluation of the function \f$f\f$:
/// \note \f$\int_{-1}^{+1} f(x) dx = \sum_{i=1}^n w_i f(x_i)\f$
/// \note n is the number of points used to discretized the integral
/// \note \f$x_i\f$ is the point to evaluate the function \f$f\f$ with
/// \note \f$w_i\f$ is the weight to assign to the function evaluation at the given point \f$x_i\f$
extern std::array<gaussQuadraturePoint, 1> gaussQuadrature1Point;

/// 2-points Gauss-Legendre quadrature {<x_1, w_1>, <x_2, w_2>}
/// \note Gauss-Legendre quadrature numerically evaluates the integral of a function \f$f\f$ with a finite sum
/// using some weights and specific points of evaluation of the function \f$f\f$:
/// \note \f$\int_{-1}^{+1} f(x) dx = \sum_{i=1}^n w_i f(x_i)\f$
/// \note n is the number of points used to discretized the integral
/// \note \f$x_i\f$ is the point to evaluate the function \f$f\f$ with
/// \note \f$w_i\f$ is the weight to assign to the function evaluation at the given point \f$x_i\f$
extern std::array<gaussQuadraturePoint, 2> gaussQuadrature2Points;

/// 3-points Gauss-Legendre quadrature {<x_1, w_1>, <x_2, w_2>, <x_3, w_3>}
/// \note Gauss-Legendre quadrature numerically evaluates the integral of a function \f$f\f$ with a finite sum
/// using some weights and specific points of evaluation of the function \f$f\f$:
/// \note \f$\int_{-1}^{+1} f(x) dx = \sum_{i=1}^n w_i f(x_i)\f$
/// \note n is the number of points used to discretized the integral
/// \note \f$x_i\f$ is the point to evaluate the function \f$f\f$ with
/// \note \f$w_i\f$ is the weight to assign to the function evaluation at the given point \f$x_i\f$
extern std::array<gaussQuadraturePoint, 3> gaussQuadrature3Points;

/// 4-points Gauss-Legendre quadrature {<x_1, w_1>, <x_2, w_2>, <x_3, w_3>, <x_4, w_4>}
/// \note Gauss-Legendre quadrature numerically evaluates the integral of a function \f$f\f$ with a finite sum
/// using some weights and specific points of evaluation of the function \f$f\f$:
/// \note \f$\int_{-1}^{+1} f(x) dx = \sum_{i=1}^n w_i f(x_i)\f$
/// \note n is the number of points used to discretized the integral
/// \note \f$x_i\f$ is the point to evaluate the function \f$f\f$ with
/// \note \f$w_i\f$ is the weight to assign to the function evaluation at the given point \f$x_i\f$
extern std::array<gaussQuadraturePoint, 4> gaussQuadrature4Points;

/// 5-points Gauss-Legendre quadrature {<x_1, w_1>, <x_2, w_2>, <x_3, w_3>, <x_4, w_4>, <x_5, w_5>}
/// \note Gauss-Legendre quadrature numerically evaluates the integral of a function \f$f\f$ with a finite sum
/// using some weights and specific points of evaluation of the function \f$f\f$:
/// \note \f$\int_{-1}^{+1} f(x) dx = \sum_{i=1}^n w_i f(x_i)\f$
/// \note n is the number of points used to discretized the integral
/// \note \f$x_i\f$ is the point to evaluate the function \f$f\f$ with
/// \note \f$w_i\f$ is the weight to assign to the function evaluation at the given point \f$x_i\f$
extern std::array<gaussQuadraturePoint, 5> gaussQuadrature5Points;

};  // namespace Math
};  // namespace SurgSim

#endif  // SURGSIM_MATH_GAUSSLEGENDREQUADRATURE_H
