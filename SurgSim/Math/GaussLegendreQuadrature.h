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

/// 1D Gauss-Legendre quadrature
struct gaussQuadraturePoint
{
	gaussQuadraturePoint(double p, double w) : point(p), weight(w){}

	const double point;
	const double weight;
};

/// 2D Gauss-Legendre quadrature on a triangle
/// \note In a triangle ABC, a point \f$P\f$ is defined by its parametrized coordinate \f$(\xi, \eta)\f$ as
/// \note \f$P = A + \xi.AB + \eta.AC\f$
struct gaussQuadratureTrianglePoint
{
	gaussQuadratureTrianglePoint(double xi, double eta, double w) : coordinateXi(xi), coordinateEta(eta), weight(w){}

	const double coordinateXi, coordinateEta; //< Parametrized coordinates xi >= 0, eta >= 0 and xi + eta <= 1.0
	const double weight;
};

/// 1D 1-point Gauss-Legendre quadrature \f${<x_1, w_1>}\f$
/// \note Gauss-Legendre quadrature numerically evaluates the integral of a function \f$f\f$ with a finite sum
/// using some weights and specific points of evaluation of the function \f$f\f$:
/// \note \f$\int_{-1}^{+1} f(x) dx = \sum_{i=1}^n w_i f(x_i)\f$
/// \note n is the number of points used to discretized the integral
/// \note \f$x_i\f$ is the point to evaluate the function \f$f\f$ with
/// \note \f$w_i\f$ is the weight to assign to the function evaluation at the given point \f$x_i\f$
extern std::array<gaussQuadraturePoint, 1> gaussQuadrature1Point;

/// 1D 2-points Gauss-Legendre quadrature \f${<x_1, w_1>, <x_2, w_2>}\f$
/// \note Gauss-Legendre quadrature numerically evaluates the integral of a function \f$f\f$ with a finite sum
/// using some weights and specific points of evaluation of the function \f$f\f$:
/// \note \f$\int_{-1}^{+1} f(x) dx = \sum_{i=1}^n w_i f(x_i)\f$
/// \note n is the number of points used to discretized the integral
/// \note \f$x_i\f$ is the point to evaluate the function \f$f\f$ with
/// \note \f$w_i\f$ is the weight to assign to the function evaluation at the given point \f$x_i\f$
extern std::array<gaussQuadraturePoint, 2> gaussQuadrature2Points;

/// 1D 3-points Gauss-Legendre quadrature \f${<x_1, w_1>, <x_2, w_2>, <x_3, w_3>}\f$
/// \note Gauss-Legendre quadrature numerically evaluates the integral of a function \f$f\f$ with a finite sum
/// using some weights and specific points of evaluation of the function \f$f\f$:
/// \note \f$\int_{-1}^{+1} f(x) dx = \sum_{i=1}^n w_i f(x_i)\f$
/// \note n is the number of points used to discretized the integral
/// \note \f$x_i\f$ is the point to evaluate the function \f$f\f$ with
/// \note \f$w_i\f$ is the weight to assign to the function evaluation at the given point \f$x_i\f$
extern std::array<gaussQuadraturePoint, 3> gaussQuadrature3Points;

/// 1D 4-points Gauss-Legendre quadrature \f${<x_1, w_1>, <x_2, w_2>, <x_3, w_3>, <x_4, w_4>}\f$
/// \note Gauss-Legendre quadrature numerically evaluates the integral of a function \f$f\f$ with a finite sum
/// using some weights and specific points of evaluation of the function \f$f\f$:
/// \note \f$\int_{-1}^{+1} f(x) dx = \sum_{i=1}^n w_i f(x_i)\f$
/// \note n is the number of points used to discretized the integral
/// \note \f$x_i\f$ is the point to evaluate the function \f$f\f$ with
/// \note \f$w_i\f$ is the weight to assign to the function evaluation at the given point \f$x_i\f$
extern std::array<gaussQuadraturePoint, 4> gaussQuadrature4Points;

/// 1D 5-points Gauss-Legendre quadrature \f${<x_1, w_1>, <x_2, w_2>, <x_3, w_3>, <x_4, w_4>, <x_5, w_5>}\f$
/// \note Gauss-Legendre quadrature numerically evaluates the integral of a function \f$f\f$ with a finite sum
/// using some weights and specific points of evaluation of the function \f$f\f$:
/// \note \f$\int_{-1}^{+1} f(x) dx = \sum_{i=1}^n w_i f(x_i)\f$
/// \note n is the number of points used to discretized the integral
/// \note \f$x_i\f$ is the point to evaluate the function \f$f\f$ with
/// \note \f$w_i\f$ is the weight to assign to the function evaluation at the given point \f$x_i\f$
extern std::array<gaussQuadraturePoint, 5> gaussQuadrature5Points;

/// 1D 100-points Gauss-Legendre quadrature \f${<x_1, w_1>, <x_2, w_2>, <x_3, w_3>, ..., <x_{100}, w_{100}>}\f$
/// \note Gauss-Legendre quadrature numerically evaluates the integral of a function \f$f\f$ with a finite sum
/// using some weights and specific points of evaluation of the function \f$f\f$:
/// \note \f$\int_{-1}^{+1} f(x) dx = \sum_{i=1}^n w_i f(x_i)\f$
/// \note n is the number of points used to discretized the integral
/// \note \f$x_i\f$ is the point to evaluate the function \f$f\f$ with
/// \note \f$w_i\f$ is the weight to assign to the function evaluation at the given point \f$x_i\f$
extern std::array<gaussQuadraturePoint, 100> gaussQuadrature100Points;

/// 2D triangle Gauss-Legendre quadrature 6-points \f${<\xi_1, \eta_1, w_1>, ..., <\xi_6, \eta_6, w_6>}\f$
/// \note Gauss-Legendre quadrature numerically evaluates the integral of a function \f$f(\xi, \eta)\f$ with a
/// finite sum using some weights and specific points on the triangle.
/// \note \f$\int_{0}^{1} \int_{0}^{1-\eta} f(\xi, \eta) d\xi d\eta = \sum_{i=1}^n w_i f(\xi_i, \eta_i)\f$
/// \note n is the number of points used to discretized the integral
/// \note \f$(\xi_i, \eta_i)\f$ is the parametrized location of the triangle point to evaluate the function with
/// \note \f$w_i\f$ is the weight to assign to the function evaluation at the given point
/// \note A 6-points Gauss-Legendre quadrature on the triangle is exact for polynomial functions of degree 4 or less.
extern std::array<gaussQuadratureTrianglePoint, 6> gaussQuadrature2DTriangle6Points;

/// 2D triangle Gauss-Legendre quadrature 12-points \f${<\xi_1, \eta_1, w_1>, ..., <\xi_{12}, \eta_{12}, w_{12}>}\f$
/// \note Gauss-Legendre quadrature numerically evaluates the integral of a function \f$f(\xi, \eta)\f$ with a
/// finite sum using some weights and specific points on the triangle.
/// \note \f$\int_{0}^{1} \int_{0}^{1-\eta} f(\xi, \eta) d\xi d\eta = \sum_{i=1}^n w_i f(\xi_i, \eta_i)\f$
/// \note n is the number of points used to discretized the integral
/// \note \f$(\xi_i, \eta_i)\f$ is the parametrized location of the triangle point to evaluate the function with
/// \note \f$w_i\f$ is the weight to assign to the function evaluation at the given point
/// \note A 12-points Gauss-Legendre quadrature on the triangle is exact for polynomial functions of degree 6 or less.
extern std::array<gaussQuadratureTrianglePoint, 12> gaussQuadrature2DTriangle12Points;

};  // namespace Math
};  // namespace SurgSim

#endif  // SURGSIM_MATH_GAUSSLEGENDREQUADRATURE_H
