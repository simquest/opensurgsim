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

#include "SurgSim/Collision/SegmentMeshTriangleMeshContact.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/DataStructures/AabbTreeNode.h"
#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SegmentMeshShape.h"

using SurgSim::DataStructures::Location;
using SurgSim::DataStructures::TriangleMesh;
using SurgSim::Math::MeshShape;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::SegmentMeshShape;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

std::pair<int, int> SegmentMeshTriangleMeshContact::getShapeTypes()
{
	return std::pair<int, int>(SurgSim::Math::SHAPE_TYPE_SEGMENTMESH, SurgSim::Math::SHAPE_TYPE_MESH);
}

std::list<std::shared_ptr<Contact>> SegmentMeshTriangleMeshContact::calculateDcdContact(
									 const Math::SegmentMeshShape& segmentMeshShape,
									 const Math::RigidTransform3d& segmentMeshPose,
									 const Math::MeshShape& triangleMeshShape,
									 const Math::RigidTransform3d& triangleMeshPose) const
{
	std::list<std::shared_ptr<Contact>> contacts;

	std::list<SurgSim::DataStructures::AabbTree::TreeNodePairType> intersectionList
		= segmentMeshShape.getAabbTree()->spatialJoin(*triangleMeshShape.getAabbTree());

	double radius = segmentMeshShape.getRadius();
	double depth = 0.0;
	Vector3d normal;
	Vector3d penetrationPointCapsule, penetrationPointTriangle, penetrationPointCapsuleAxis;

	for (const auto& intersection : intersectionList)
	{
		std::shared_ptr<SurgSim::DataStructures::AabbTreeNode> nodeSegment = intersection.first;
		std::shared_ptr<SurgSim::DataStructures::AabbTreeNode> nodeTriangle = intersection.second;

		std::list<size_t> edgeList;
		std::list<size_t> triangleList;

		nodeSegment->getIntersections(nodeTriangle->getAabb(), &edgeList);
		nodeTriangle->getIntersections(nodeSegment->getAabb(), &triangleList);

		for (auto i = triangleList.begin(); i != triangleList.end(); ++i)
		{
			const Vector3d& normalTriangle = triangleMeshShape.getNormal(*i);
			if (normalTriangle.isZero())
			{
				continue;
			}

			const auto& verticesTriangle = triangleMeshShape.getTrianglePositions(*i);

			for (auto j = edgeList.begin(); j != edgeList.end(); ++j)
			{
				const auto& verticesSegment = segmentMeshShape.getEdgePositions(*j);

				// Check if the triangle and capsule intersect.
				if (SurgSim::Math::calculateContactTriangleCapsule(
						verticesTriangle[0], verticesTriangle[1], verticesTriangle[2], normalTriangle,
						verticesSegment[0], verticesSegment[1], radius,
						&depth, &penetrationPointTriangle, &penetrationPointCapsule, &normal,
						&penetrationPointCapsuleAxis))
				{
					// Create the contact.
					std::pair<Location, Location> penetrationPoints;
					SurgSim::Math::Vector2d barycentricCoordinate2;
					SurgSim::Math::barycentricCoordinates(penetrationPointCapsuleAxis,
														  verticesSegment[0], verticesSegment[1], &barycentricCoordinate2);
					penetrationPoints.first.elementMeshLocalCoordinate.setValue(
						SurgSim::DataStructures::IndexedLocalCoordinate(*j, barycentricCoordinate2));
					penetrationPoints.first.rigidLocalPosition.setValue(segmentMeshPose.inverse() *penetrationPointCapsuleAxis);

					Vector3d barycentricCoordinate;
					SurgSim::Math::barycentricCoordinates(penetrationPointTriangle,
														  verticesTriangle[0], verticesTriangle[1], verticesTriangle[2], normalTriangle,
														  &barycentricCoordinate);

					penetrationPoints.second.triangleMeshLocalCoordinate.setValue(
						SurgSim::DataStructures::IndexedLocalCoordinate(*i, barycentricCoordinate));
					penetrationPoints.second.rigidLocalPosition.setValue(triangleMeshPose.inverse() * penetrationPointTriangle);

					// Create the contact.
					contacts.push_back(std::make_shared<Contact>(COLLISION_DETECTION_TYPE_DISCRETE,
									   std::abs(depth) + (penetrationPointCapsule - penetrationPointCapsuleAxis).dot(normal),
									   1.0, Vector3d::Zero(), -normal, penetrationPoints));
				}
			}
		}
	}

	return contacts;
}

// a1.x + a0 = 0
int solveLinearEquation(double a0, double a1, double* results)
{
	if (a1 == 0.0)
	{
		return 0;
	}
	results[0] = -a0 / a1;
	return 1;
}

// a2.x^2 + a1.x + a0 = 0
int solveQuadraticEquation(double a0, double a1, double a2, double* results)
{
	if (a2 == 0.0)
	{
		return solveLinearEquation(a0, a1, results);
	}

	double determinant = a1 * a1 - 4.0 * a0 * a2;
	if (determinant < 0.0)
	{
		return 0;
	}
	else if (determinant == 0.0)
	{
		results[0] = -a1 / (2.0 * a2);
		return 1;
	}
	double tmp = std::sqrt(determinant);
	results[0] = (-a1 - tmp) / (2.0 * a2);
	results[1] = (-a1 + tmp) / (2.0 * a2);
	return 2;
}

/// a3.x^3 + a2.x^2 + a1.x + a0 = 0
/// https://en.wikipedia.org/wiki/Cubic_function#General_formula_for_roots
int solveCubicEquationGeneralMethod(double a0, double a1, double a2, double a3, double* results)
{
	const double& a = a3;
	const double& b = a2;
	const double& c = a1;
	const double& d = a0;
	double aSQ = a * a; 
	double bSQ = b * b;
	double bCU = bSQ * b;

	int nResults = 0;

	if (a3 == 0.0)
	{
		return solveQuadraticEquation(a0, a1, a2, results);
	}

	double delta0 = bSQ - 3.0 * a * c;
	double delta1 = 2.0 * bCU - 9.0 * a * b * c + 27.0 * aSQ * d;
	double tmp = delta1 * delta1 - 4.0 * delta0 * delta0 * delta0;
	double delta = -tmp / (27.0 * aSQ);
	double C = pow((delta1 + std::sqrt(tmp)) * 0.5, 1.0 / 3.0);

	double x = -1.0 / (3.0 * a) * (b + C + delta0 / C);
	return nResults;
}

/// a3.x^3 + a2.x^2 + a1.x + a0 = 0
/// https://en.wikipedia.org/wiki/Cubic_function#Cardano.27s_method
int solveCubicEquationCardanoMethod(double a0, double a1, double a2, double a3, double* results)
{
	int nResults = 0;

	if (a3 == 0.0)
	{
		return solveQuadraticEquation(a0, a1, a2, results);
	}

	/// Using the variable change x = t - a2/(3*a3), we form the depressed cubic equation t^3 + p.t + q = 0
	double p = (3.0 * a3 * a1 - a2 * a2) / (3.0 * a3 * a3);
	double q = (2.0 * a2 * a2 * a2 - 9.0 * a3 * a2 * a1 + 27.0 * a3 * a3 * a0) / (27.0 * a3 * a3 * a3);

	// Using the Cardano's substitution t = u + v, we have the equation
	// u^3 + v^3 + (3uv + p)(u + v) + q = 0
	// Let's impose 3uv + p = 0
	// We get { u^3v^3 = -p^3/27
	//        { u^3 + v^3 = -q
	// Thus u^3 and v^3 are the roots of the quadratic equation z^2 + qz - p^3/27 = 0

	if (p == 0.0 && q == 0.0)
	{
		double t = 0.0;
		double x = t - a2 / (3.0 * a3);
		results[nResults++] = x;
	}
	else if (p == 0.0)
	{
		double u = -pow(q, 1.0 / 3.0);
		double v = 0.0;
		double t = u + v;
		double x = t - a2 / (3.0 * a3);
		results[nResults++] = x;
	}
	else if (q == 0.0)
	{
		double t = 0.0;
		double x = t - a2 / (3.0 * a3);
		results[nResults++] = x;

		if (p < 0.0)
		{
			t = std::sqrt(-p);
			x = t - a2 / (3.0 * a3);
			results[nResults++] = x;

			t = -t;
			x = t - a2 / (3.0 * a3);
			results[nResults++] = x;
		}
	}
	else
	{

	}

	return nResults;
}

/// a3.x^3 + a2.x^2 + a1.x + a0 = 0
/// https://en.wikipedia.org/wiki/Cubic_function#Vieta.27s_substitution
int solveCubicEquationVietaMethod(double a0, double a1, double a2, double a3, double* results)
{
	int nResults = 0;

	if (a3 == 0.0)
	{
		return solveQuadraticEquation(a0, a1, a2, results);
	}

	/// Using the variable change x = t - a2/(3*a3), we form the depressed cubic equation t^3 + p.t + q = 0
	double p = (3.0 * a3 * a1 - a2 * a2) / (3.0 * a3 * a3);
	double q = (2.0 * a2 * a2 * a2 - 9.0 * a3 * a2 * a1 + 27.0 * a3 * a3 * a0) / (27.0 * a3 * a3 * a3);

	// Using the Vieta's substitution t = w - p/(3*w), we have the equation
	// w^3 + q - p^3/(27*w^3) = 0
	// Multiplying by w^3, becomes:
	// (w^3)^2 + w^3.q - p^3/27 = 0 (a quadratic equation in w^3).
	double resultsQuadratic[2];
	int nResultsQuadratic = solveQuadraticEquation(1.0, q, -p * p * p / 27.0, resultsQuadratic);
	for (int i = 0; i < nResultsQuadratic; i++)
	{
		double w = std::pow(resultsQuadratic[i], 1.0 / 3.0); // Only a single cube root per real number
		double t = w - p / (3.0 * w);
		double x = t - a2 / (3.0 * a3);

		results[nResults] = x;
		nResults++;
	}

	return nResults;
}

double evaluateCubicEquation(double a0, double a1, double a2, double a3, double x)
{
	return x*x*x * a3 + x*x * a2 + x * a1 + a0;
}

double evaluateCubicEquationDerivative(double a0, double a1, double a2, double a3, double x)
{
	return 3.0*x*x * a3 + 2.0*x * a2 + a1;
}

/// a3.x^3 + a2.x^2 + a1.x + a0 = 0
// f(x) = 0
// Newton-Raphson f(x+h) = f(x) + h.f'(x) = 0 <=> h = -f(x)/f'(x)
// After finding 1 solution, we can factorize and solve the roots for remaining quadratic equation.
int solveCubicEquationNewtonRaphson(double a0, double a1, double a2, double a3, double* result)
{
	double x0 = 0.0, h;
	do
	{
		double f = evaluateCubicEquation(a0, a1, a2, a3, x0);
		double fprime = evaluateCubicEquationDerivative(a0, a1, a2, a3, x0);
		h = -f / fprime;
		x0 += h;
	} while (std::abs(h / x0) < 1e-10);

	// Equation can therefore be written: a3.(x - x0)(x^2 + q.x - a0/(a3.x0)) = 0
	// a3x^3 + a3.q.x^2 - a3.x.a0/(a3.x0) - a3.x0.x^2 - a3.x0.q.x + a3.x0.a0/(a3.x0) = 0
	// a3x^3 + a3.(q - x0).x^2 - (a0/x0 + a3.x0.q).x + a0 = 0
	// {a3.(q-x0) = a2 <=> q = a2/a3 + x0
	// {- (a0/x0 + a3.x0.q) = a1 <=> q = (-a1 - a0/x0) / (a3.x0)
	//*result = x;
	return true;
}

// Solve numerically a cubic equation in a given range, returning a single solution if any, the smallest one
// f(x) = 0
// Newton-Raphson f(x+h) = f(x) + h.f'(x) = 0 <=> h = -f(x)/f'(x)
bool solveCubicEquationMinimumRootInRange(double a0, double a1, double a2, double a3, double min, double max, double* result)
{
	//std::cout << "solveCubicEquationMinimumRootInRange [" << min << ", " << max << "]" << std::endl;
	do
	{
		//std::cout << "  [" << min << ", " << max << "]" << std::endl;

		double middle = (min + max) * 0.5;

		if (std::abs(max - min) < 1e-15)
		{
			*result = middle;
			return true;
		}

		double evalMin = evaluateCubicEquation(a0, a1, a2, a3, min);
		double evalMinDerivative = evaluateCubicEquationDerivative(a0, a1, a2, a3, min);

		double evalMax = evaluateCubicEquation(a0, a1, a2, a3, max);
		double evalMaxDerivative = evaluateCubicEquationDerivative(a0, a1, a2, a3, max);

		double evalMiddle = evaluateCubicEquation(a0, a1, a2, a3, middle);
		double evalMiddleDerivative = evaluateCubicEquationDerivative(a0, a1, a2, a3, middle);

		// Found the first root at the range border...
		if (std::abs(evalMin) < 1e-15)
		{
			//std::cout << " eval(min) = " << evalMin << std::endl;
			*result = min;
			return true;
		}

		// Do we have a solution between min and middle ?
		if (evalMin * evalMiddle < 0.0)
		{
			max = middle;
			continue;
		}
		else
		{
			// Change of direction in between [min, middle], a root could be hidding in the interval.
			if (evalMinDerivative * evalMiddleDerivative < 0.0)
			{
				//std::cout << "  Same sign at min and middle, but change of tangent, looking into [" << min << ", " << middle << "]" << std::endl;

				// Check on the interval [min, middle] for any root
				if (solveCubicEquationMinimumRootInRange(a0, a1, a2, a3, min, middle, result))
				{
					return true;
				}
			}

			// Here, we need to look in [middle, max]
			if (evalMiddle * evalMax < 0.0)
			{
				min = middle;
			}
			else
			{
				// Change of direction in between [middle, max], a root could be hidding in the interval.
				if (evalMiddleDerivative * evalMaxDerivative < 0.0)
				{
					//std::cout << "  Same sign at middle and max, but change of tangent, looking into [" << middle << ", " << max << "]" << std::endl;

					// Check on the interval [middle, max] for any root
					if (solveCubicEquationMinimumRootInRange(a0, a1, a2, a3, middle, max, result))
					{
						return true;
					}
				}

				// At this point, we have not found a solution in between [min, middle], neither in [middle, max], we don't have a solution
				break;
			}
		}
	}
	while (true);

	return false;
}

/// a3.x^3 + a2.x^2 + a1.x + a0 = 0
int solveCubicEquationNumerically(double a0, double a1, double a2, double a3, double min, double max, double* results)
{
	int nResults = 0;

	if (a3 == 0.0)
	{
		return solveQuadraticEquation(a0, a1, a2, results);
	}

	nResults = solveCubicEquationMinimumRootInRange(a0, a1, a2, a3, 0.0, 1.0, results) == true ? 1 : 0;

	return nResults;
}

int solveCubicEquation(double a0, double a1, double a2, double a3, double* results)
{
	//const double& a = a3;
	//const double& b = a2;
	//const double& c = a1;
	//const double& d = a0;
	//double discriminant = 18.0 * a * b * c * d - 4.0 * b*b*b * d + b*b * c*c - 4.0 * a * c*c*c - 27.0 * a*a * d*d;

	int nResults = solveCubicEquationNumerically(a0, a1, a2, a3, 0.0, 1.0, results);
	//int nResults = solveCubicEquationVietaMethod(a0, a1, a2, a3, results);
	//std::cout << "Vieta     found: ";
	//for (int i = 0; i < nResults; i++)
	//{
	//	std::cout << results[i] << " ";
	//}
	//std::cout << std::endl;
	//for (int i = 0; i < nResults; i++)
	//{
	//	double x = results[i];
	//	double tmp = a3 * (x*x*x) + a2 * (x*x) + a1 * x + a0;
	//	if (std::abs(tmp) > 1e-10)
	//	{
	//		std::cerr << "root     " << x << " is not a solution to the cubic equation, it evaluates to " << tmp << " and not 0" << std::endl;
	//	}
	//}

	//double resultsNumerical[3];
	//int nResultsNumerical = solveCubicEquationNumerically(a0, a1, a2, a3, 0.0, 1.0, resultsNumerical);
	//std::cout << "Numerical found: ";
	//for (int i = 0; i < nResultsNumerical; i++)
	//{
	//	std::cout << resultsNumerical[i] << " ";
	//}
	//std::cout << std::endl;
	//for (int i = 0; i < nResultsNumerical; i++)
	//{
	//	double x = resultsNumerical[i];
	//	double tmp = a3 * (x*x*x) + a2 * (x*x) + a1 * x + a0;
	//	if (std::abs(tmp) > 1e-10)
	//	{
	//		std::cerr << "root numerical " << x << " is not a solution to the cubic equation, it evaluates to " << tmp << " and not 0" << std::endl;
	//	}
	//}


	//if (discriminant > 0.0 && nResults != 3)
	//{
	//	std::cerr << "This equation should have 3 real roots, but we found only " << nResults << " roots" << std::endl;
	//}
	//else if (discriminant == 0.0 && nResults == 0)
	//{
	//	std::cerr << "This equation should have a multiple root and all its roots are real, but we found " << nResults << " roots" << std::endl;
	//}
	//else if (discriminant < 0.0 && nResults != 1)
	//{
	//	std::cerr << "This equation should have exactly 1 real root, but we found " << nResults << " roots" << std::endl;
	//}

	return nResults;
}

bool isCoplanarPointInsideTriangleAtTOI(
	double potentialTimeOfImpact,
	const std::pair<Math::Vector3d, Math::Vector3d>& P,
	const std::pair<Math::Vector3d, Math::Vector3d>& A,
	const std::pair<Math::Vector3d, Math::Vector3d>& B,
	const std::pair<Math::Vector3d, Math::Vector3d>& C,
	Math::Vector3d* baryCoords)
{
	Math::Vector3d Pt = P.first + potentialTimeOfImpact * (P.second - P.first);
	Math::Vector3d At = A.first + potentialTimeOfImpact * (A.second - A.first);
	Math::Vector3d Bt = B.first + potentialTimeOfImpact * (B.second - B.first);
	Math::Vector3d Ct = C.first + potentialTimeOfImpact * (C.second - C.first);

	bool result = Math::barycentricCoordinates(Pt, At, Bt, Ct, baryCoords);
	return (result &&
		(*baryCoords)[0] >= -Math::Geometry::ScalarEpsilon &&
		(*baryCoords)[1] >= -Math::Geometry::ScalarEpsilon &&
		(*baryCoords)[2] >= -Math::Geometry::ScalarEpsilon);
}

/// Simple cubic-solver https://graphics.stanford.edu/courses/cs468-02-winter/Papers/Collisions_vetements.pdf
/// Optimized method http://www.robotics.sei.ecnu.edu.cn/CCDLY/GMOD15.pdf
/// Optimized method https://www.cs.ubc.ca/~rbridson/docs/brochu-siggraph2012-ccd.pdf
bool pointTriangleCcd(
	const std::pair<Math::Vector3d, Math::Vector3d>& P,
	const std::pair<Math::Vector3d, Math::Vector3d>& A,
	const std::pair<Math::Vector3d, Math::Vector3d>& B,
	const std::pair<Math::Vector3d, Math::Vector3d>& C,
	double* timeOfImpact, double* tv01Param, double* tv02Param)
{
	Math::Vector3d AP0 = P.first - A.first;
	Math::Vector3d AB0 = B.first - A.first;
	Math::Vector3d AC0 = C.first - A.first;
	Math::Vector3d VA = (A.second - A.first);
	Math::Vector3d VAP = (P.second - P.first) - VA;
	Math::Vector3d VAB = (B.second - B.first) - VA;
	Math::Vector3d VAC = (C.second - C.first) - VA;
	double a0 = AP0.dot(AB0.cross(AC0));
	double a1 = AP0.dot(AB0.cross(VAC) + VAB.cross(AC0)) + VAP.dot(AB0.cross(AC0));
	double a2 = AP0.dot(VAB.cross(VAC)) + VAP.dot(AB0.cross(VAC) + VAB.cross(AC0));
	double a3 = VAP.dot(VAB.cross(VAC));
	/// P(t) = (1-t) * P0 + t * P1 = P0 + t * [P1 - P0] = P0 + t * VP with VP = P1 - P0
	/// Similarily for A(t), B(t) and C(t)
	/// Therefore we have AP(t) = P(t) - A(t) = P(0) + t * VP - A(0) - t * VA
	///                         = AP(0) + t * [VP - VA] = AP(0) + t * VAP
	///
	/// AP(t).[AB(t).cross(AC(t))] = 0 means that the 4 points are coplanar, so potentially intersecting.
	/// [AP(0) + t*VAP] . [AB(0).cross(AC(0) + t*VAC) + t*VAB.cross(AC(0) + t*VAC)] = 0
	/// AP(0) . [AB(0).cross(AC(0) + t*VAC) + t*VAB.cross(AC(0) + t*VAC)] +
	/// t*VAP . [AB(0).cross(AC(0) + t*VAC) + t*VAB.cross(AC(0) + t*VAC)] = 0
	/// AP(0).AB(0).cross(AC(0)) + t  *AP(0).AB(0).cross(VAC) + t  *AP(0).VAB.cross(AC(0)) + t^2*AP(0).VAB.cross(VAC) +
	/// t*VAP.AB(0).cross(AC(0)) + t^2*  VAP.AB(0).cross(VAC) + t^2*  VAP.VAB.cross(AC(0)) + t^3*  VAP.VAB.cross(VAC) = 0
	/// AP(0).AB(0).cross(AC(0)) +
	/// t * [AP(0).AB(0).cross(VAC) + AP(0).VAB.cross(AC(0)) + VAP.AB(0).cross(AC(0))] +
	/// t^2 * [AP(0).VAB.cross(VAC) + VAP.AB(0).cross(VAC) + VAP.VAB.cross(AC(0))] +
	/// t^3 * [VAP.VAB.cross(VAC)] = 0
	Math::Vector3d roots;
	Math::Vector3d baryCoords;
	int nRoots = solveCubicEquation(a0, a1, a2, a3, roots.data());
	*timeOfImpact = std::numeric_limits<double>::max();
	for (int i = 0; i < nRoots; i++)
	{
		if (roots[i] >= 0.0 && roots[i] <= 1.0 && roots[i] < *timeOfImpact && isCoplanarPointInsideTriangleAtTOI(roots[i], P, A, B, C, &baryCoords))
		{
			*timeOfImpact = roots[i];
			*tv01Param = baryCoords[1];
			*tv02Param = baryCoords[2];
		}
	}

	if (*timeOfImpact == std::numeric_limits<double>::max())
	{
		return false;
	}

	SURGSIM_ASSERT(*timeOfImpact >= 0.0 && *timeOfImpact <= 1.0);
	SURGSIM_ASSERT(*tv01Param >= 0.0);
	SURGSIM_ASSERT(*tv02Param >= 0.0);
	SURGSIM_ASSERT(*tv01Param + *tv02Param <= 1.0);

	return true;
}

bool areCoplanarSegmentsIntersectingAtTOI(
	double potentialTimeOfImpact,
	const std::pair<Math::Vector3d, Math::Vector3d>& A,
	const std::pair<Math::Vector3d, Math::Vector3d>& B,
	const std::pair<Math::Vector3d, Math::Vector3d>& C,
	const std::pair<Math::Vector3d, Math::Vector3d>& D,
	Math::Vector2d* baryCoords)
{
	Math::Vector3d At = A.first + potentialTimeOfImpact * (A.second - A.first);
	Math::Vector3d Bt = B.first + potentialTimeOfImpact * (B.second - B.first);
	Math::Vector3d Ct = C.first + potentialTimeOfImpact * (C.second - C.first);
	Math::Vector3d Dt = D.first + potentialTimeOfImpact * (D.second - D.first);

	// P = A + alpha.AB and P = C + beta.CD
	// A + alpha.AB = C + beta.CD
	// (AB -CD).(alpha) = (AC) which is a 3x2 linear system A.x = b
	//          (beta )
	// Let's solve it using (A^t.A)^-1.(A^t.A) = Id2x2
	// (A^t.A)^-1.A^t.(A.x) = (A^t.A)^-1.A^t.b
	// x = (A^t.A)^-1.A^t.b

	Eigen::Matrix<double, 3, 2> matrixA;
	matrixA.col(0) = Bt - At;
	matrixA.col(1) = -(Dt - Ct);

	Math::Vector3d b = Ct - At;
	Math::Matrix22d inv;
	bool invertible;
	(matrixA.transpose() * matrixA).computeInverseWithCheck(inv, invertible);
	if (!invertible)
	{
		return false;
	}

	*baryCoords = inv * matrixA.transpose() * b;
	return (*baryCoords)[0] >= 0.0 && (*baryCoords)[0] <= 1.0 && (*baryCoords)[1] >= 0.0 && (*baryCoords)[1] <= 1.0;
}

bool segmentSegmentCcd(
	const std::pair<Math::Vector3d, Math::Vector3d>& A,
	const std::pair<Math::Vector3d, Math::Vector3d>& B,
	const std::pair<Math::Vector3d, Math::Vector3d>& C,
	const std::pair<Math::Vector3d, Math::Vector3d>& D,
	double* timeOfImpact, double* s0p1Factor, double* s1p1Factor)
{
	Math::Vector3d AB0 = B.first - A.first;
	Math::Vector3d AC0 = C.first - A.first;
	Math::Vector3d CD0 = D.first - C.first;
	Math::Vector3d VA = (A.second - A.first);
	Math::Vector3d VC = (C.second - C.first);
	Math::Vector3d VAB = (B.second - B.first) - VA;
	Math::Vector3d VAC = VC - VA;
	Math::Vector3d VCD = (D.second - D.first) - VC;
	double a0 = AB0.cross(CD0).dot(AC0);
	double a1 = AB0.cross(CD0).dot(VAC) + (AB0.cross(VCD) + VAB.cross(CD0)).dot(AC0);
	double a2 = (AB0.cross(VCD) + VAB.cross(CD0)).dot(VAC) + VAB.cross(VCD).dot(AC0);
	double a3 = VAB.cross(VCD).dot(VAC);
	/// A(t) = (1-t) * A0 + t * A1 = A0 + t * [A1 - A0] = A0 + t * VA with VA = A1 - A0
	/// Similarily for B(t), C(t) and D(t)
	/// Therefore we have AB(t) = B(t) - A(t) = B(0) + t * VB - A(0) - t * VA
	///                         = AB(0) + t * [VB - VA] = AB(0) + t * VAB
	///
	/// [AB(t).cross(CD(t))].AC(t) = 0 means that the 4 points are coplanar, so potentially intersecting.
	/// [AB(0).cross(CD(0)) + t*AB(0).cross(VCD) + t*VAB.cross(CD(0)) + t^2*VAB.cross(VCD)] . [AC(0) + t * VAC] = 0
	/// t^0 * [[AB(0).cross(CD(0))].AC(0)] +
	/// t^1 * [[AB(0).cross(CD(0))].VAC + [AB(0).cross(VCD)].AC(0) + [VAB.cross(CD(0))].AC(0)] +
	/// t^2 * [[AB(0).cross(VCD)].VAC + [VAB.cross(CD(0))].VAC + [VAB.cross(VCD)].AC(0)] +
	/// t^3 * [[VAB.cross(VCD)].VAC] = 0
	Math::Vector3d roots;
	Math::Vector2d baryCoords;
	int nRoots = solveCubicEquation(a0, a1, a2, a3, roots.data());
	*timeOfImpact = std::numeric_limits<double>::max();
	for (int i = 0; i < nRoots; i++)
	{
		if (roots[i] >= 0.0 && roots[i] <= 1.0 && roots[i] < *timeOfImpact && areCoplanarSegmentsIntersectingAtTOI(roots[i], A, B, C, D, &baryCoords))
		{
			*timeOfImpact = roots[i];
			*s0p1Factor = baryCoords[0];
			*s1p1Factor = baryCoords[1];
		}
	}

	if (*timeOfImpact == std::numeric_limits<double>::max())
	{
		return false;
	}

	SURGSIM_ASSERT(*timeOfImpact >= 0.0 && *timeOfImpact <= 1.0);
	SURGSIM_ASSERT(*s0p1Factor >= 0.0 && *s0p1Factor <= 1.0);
	SURGSIM_ASSERT(*s1p1Factor >= 0.0 && *s1p1Factor <= 1.0);

	return true;
}

bool isThisContactADuplicate(const std::shared_ptr<Contact>& newContact, const std::list<std::shared_ptr<Contact>>& contacts)
{
	for (const auto& contact : contacts)
	{
		// Same type? time? location? and normal?
		if (contact->type == newContact->type &&
			std::abs(contact->time - newContact->time) < 1e-8 &&
			contact->penetrationPoints.first.isApprox(newContact->penetrationPoints.first) &&
			contact->penetrationPoints.second.isApprox(newContact->penetrationPoints.second) &&
			(contact->normal - newContact->normal).squaredNorm() < 1e-8)
		{
			return true;
		}
	}

	return false;
}

std::list<std::shared_ptr<Contact>> SegmentMeshTriangleMeshContact::calculateCcdContact(
	const Math::SegmentMeshShape& shape1AtTime0, const Math::RigidTransform3d& pose1AtTime0,
	const Math::SegmentMeshShape& shape1AtTime1, const Math::RigidTransform3d& pose1AtTime1,
	const Math::MeshShape& shape2AtTime0, const Math::RigidTransform3d& pose2AtTime0,
	const Math::MeshShape& shape2AtTime1, const Math::RigidTransform3d& pose2AtTime1) const
{
	//bool segmentMeshShapeMoved = false;
	//bool meshShapeMoved = false;
	std::list<std::shared_ptr<Contact>> contacts;

	SURGSIM_ASSERT(pose1AtTime0.isApprox(Math::RigidTransform3d::Identity()));
	SURGSIM_ASSERT(pose1AtTime1.isApprox(Math::RigidTransform3d::Identity()));

	//if (!pose2AtTime0.isApprox(pose2AtTime1))
	//{
	//	std::stringstream ss;
	//	ss << "SegmentMeshTriangleMeshContact::calculateCcdContact > MeshShape poses are different" << std::endl;
	//	std::cout << ss.str();
	//}

	SURGSIM_ASSERT(shape1AtTime0.getNumEdges() > 0);
	SURGSIM_ASSERT(shape1AtTime0.getNumEdges() == shape1AtTime1.getNumEdges());
	SURGSIM_ASSERT(shape2AtTime0.getNumTriangles() > 0);
	SURGSIM_ASSERT(shape2AtTime0.getNumTriangles() == shape2AtTime1.getNumTriangles());

	for (int edgeId = 0; edgeId < shape1AtTime0.getNumEdges(); edgeId++)
	{
		auto edgeT0 = shape1AtTime0.getEdge(edgeId);
		auto edgeT1 = shape1AtTime1.getEdge(edgeId);

		SURGSIM_ASSERT(edgeT0.verticesId == edgeT1.verticesId) << "Edges are different" << std::endl <<
			"(" << edgeT0.verticesId[0] << "," << edgeT0.verticesId[1] << ")" << std::endl <<
			"(" << edgeT1.verticesId[0] << "," << edgeT1.verticesId[1] << ")" << std::endl <<
			"edgeT0.valid = " << edgeT0.isValid << "; edgeT1.valid = " << edgeT1.isValid << std::endl;

		std::pair<Math::Vector3d, Math::Vector3d> sv0 = std::make_pair(shape1AtTime0.getVertexPosition(edgeT0.verticesId[0]), shape1AtTime1.getVertexPosition(edgeT1.verticesId[0]));
		std::pair<Math::Vector3d, Math::Vector3d> sv1 = std::make_pair(shape1AtTime0.getVertexPosition(edgeT0.verticesId[1]), shape1AtTime1.getVertexPosition(edgeT1.verticesId[1]));
		Math::Aabbd segmentAabb;
		segmentAabb.extend(sv0.first);
		segmentAabb.extend(sv0.second);
		segmentAabb.extend(sv1.first);
		segmentAabb.extend(sv1.second);

		if (segmentAabb.isEmpty())
		{
			std::cout << "AABB segment is empty" << std::endl;
		}

		//if (!segmentMeshShapeMoved && (!sv0.first.isApprox(sv0.second) || !sv1.first.isApprox(sv1.second)))
		//{
		//	segmentMeshShapeMoved = true;
		//}

		for (int triangleId = 0; triangleId < shape2AtTime0.getNumTriangles(); triangleId++)
		{
			auto triangleT0 = shape2AtTime0.getTriangle(triangleId);
			auto triangleT1 = shape2AtTime1.getTriangle(triangleId);

			SURGSIM_ASSERT(triangleT0.verticesId == triangleT1.verticesId) << "Triangles are different" << std::endl <<
				"(" << triangleT0.verticesId[0] << "," << triangleT0.verticesId[1] << "," << triangleT0.verticesId[2] << ")" << std::endl <<
				"(" << triangleT1.verticesId[0] << "," << triangleT1.verticesId[1] << "," << triangleT1.verticesId[2] << ")" << std::endl <<
				"triangleT0.valid = " << triangleT0.isValid << "; triangleT1.valid = " << triangleT1.isValid << std::endl;

			std::pair<Math::Vector3d, Math::Vector3d> tv0 = std::make_pair(shape2AtTime0.getVertexPosition(triangleT0.verticesId[0]), shape2AtTime1.getVertexPosition(triangleT1.verticesId[0]));
			std::pair<Math::Vector3d, Math::Vector3d> tv1 = std::make_pair(shape2AtTime0.getVertexPosition(triangleT0.verticesId[1]), shape2AtTime1.getVertexPosition(triangleT1.verticesId[1]));
			std::pair<Math::Vector3d, Math::Vector3d> tv2 = std::make_pair(shape2AtTime0.getVertexPosition(triangleT0.verticesId[2]), shape2AtTime1.getVertexPosition(triangleT1.verticesId[2]));

			Math::Aabbd triangleAabb;
			triangleAabb.extend(tv0.first);
			triangleAabb.extend(tv0.second);
			triangleAabb.extend(tv1.first);
			triangleAabb.extend(tv1.second);
			triangleAabb.extend(tv2.first);
			triangleAabb.extend(tv2.second);

			if (triangleAabb.isEmpty())
			{
				std::cout << "AABB triangle is empty" << std::endl;
			}

			//if (!meshShapeMoved && (!tv0.first.isApprox(tv0.second) || !tv1.first.isApprox(tv1.second) || !tv2.first.isApprox(tv2.second)))
			//{
			//	meshShapeMoved = true;
			//}

			if (!segmentAabb.intersects(triangleAabb))
			{
				continue;
			}

			double earliestTimeOfImpact = std::numeric_limits<double>::max();
			double segmentAlpha; // P = P0 + alpha.P0P1
			double triangleAlpha, triangleBeta; // P = P0 + alpha.P0P1 + beta.P0P2

			Math::Vector3d pt;
			Math::Vector3d tn = ((tv1.first - tv0.first).cross(tv2.first - tv0.first));
			if (tn.norm() < 1e-8)
			{
				std::cerr << "triangle normal 0" << std::endl;
			}
			tn.normalize();
			if (Math::doesCollideSegmentTriangle<Math::Vector3d::Scalar, Math::Vector3d::Options>(
					sv0.first, sv1.first,
					tv0.first, tv1.first, tv2.first,
					tn,
					&pt))
			{
				//std::cout << " COLLISION AT T=0; pt = " << pt.transpose();
				Math::Vector2d baryCoordSegment;
				Math::Vector3d baryCoordTriangle;
				if (!Math::barycentricCoordinates(pt, sv0.first, sv1.first, &baryCoordSegment) || baryCoordSegment[0] != baryCoordSegment[0])
				{
					std::cerr << "ERROR Segment barycentric coordinate!!!!" << std::endl;
				}
				//std::cout << " baryCoordSegment = " << baryCoordSegment.transpose();
				if (!Math::barycentricCoordinates(pt, tv0.first, tv1.first, tv2.first, &baryCoordTriangle) || baryCoordTriangle[0] != baryCoordTriangle[0])
				{
					std::cerr << "ERROR Triangle barycentric coordinate!!!!" << std::endl;
				}
				//std::cout << " baryCoordTriangle = " << baryCoordTriangle.transpose() << std::endl;
				earliestTimeOfImpact = 0.0;
				segmentAlpha = baryCoordSegment[1];
				triangleAlpha = baryCoordTriangle[1];
				triangleBeta = baryCoordTriangle[2];
			}
			else
			{
				// Calculate Segment/Segment ccd
				double timeOfImpact;
				double sFactor, tFactor;
				if (segmentSegmentCcd(sv0, sv1, tv0, tv1, &timeOfImpact, &sFactor, &tFactor))
				{
					if (timeOfImpact < earliestTimeOfImpact)
					{
						earliestTimeOfImpact = timeOfImpact;
						segmentAlpha = sFactor;
						triangleAlpha = tFactor;
						triangleBeta = 0.0;
					}
				}

				if (segmentSegmentCcd(sv0, sv1, tv1, tv2, &timeOfImpact, &sFactor, &tFactor))
				{
					if (timeOfImpact < earliestTimeOfImpact)
					{
						earliestTimeOfImpact = timeOfImpact;
						segmentAlpha = sFactor;
						triangleAlpha = 1.0 - tFactor; // P = P0 + P0P1.(1 - tFactor) + P0P2.tFactor
						triangleBeta = tFactor;
					}
				}

				if (segmentSegmentCcd(sv0, sv1, tv2, tv0, &timeOfImpact, &sFactor, &tFactor))
				{
					if (timeOfImpact < earliestTimeOfImpact)
					{
						earliestTimeOfImpact = timeOfImpact;
						segmentAlpha = sFactor;
						triangleAlpha = 0.0; // P = P0 + P0P2.(1 - tFactor)
						triangleBeta = 1.0 - tFactor;
					}
				}

				// Calculate Point/Triangle ccd
				double u, v;
				if (pointTriangleCcd(sv0, tv0, tv1, tv2, &timeOfImpact, &u, &v))
				{
					if (timeOfImpact < earliestTimeOfImpact)
					{
						earliestTimeOfImpact = timeOfImpact;
						segmentAlpha = 0.0;
						triangleAlpha = u;
						triangleBeta = v;
					}
				}

				if (pointTriangleCcd(sv1, tv0, tv1, tv2, &timeOfImpact, &u, &v))
				{
					if (timeOfImpact < earliestTimeOfImpact)
					{
						earliestTimeOfImpact = timeOfImpact;
						segmentAlpha = 1.0;
						triangleAlpha = u;
						triangleBeta = v;
					}
				}
			}

			// False positive from the AABB, no collision found
			if (earliestTimeOfImpact == std::numeric_limits<double>::max())
			{
				continue;
			}

			SURGSIM_ASSERT(segmentAlpha >= 0.0 && segmentAlpha <= 1.0) << "earliestTimeOfImpact = " << earliestTimeOfImpact << "; segmentAlpha = " << segmentAlpha;
			SURGSIM_ASSERT(triangleAlpha >= 0.0 && triangleBeta >= 0.0 && triangleAlpha + triangleBeta <= 1.0) <<
				"earliestTimeOfImpact = " << earliestTimeOfImpact << "; triangleAlpha = " << triangleAlpha << " triangleBeta = " << triangleBeta << " alpha + beta = " << triangleAlpha + triangleBeta;

			// Look for the contact information at t=0 (with interpenetration, contact normal,...)
			std::cout << "Seg[" << edgeId << "]/Tri[" << triangleId << "]" << " penetration depth ";
			{
				Math::Vector3d S = Math::interpolate(sv0.first, sv1.first, segmentAlpha);
				Math::Vector3d T0T1 = tv1.first - tv0.first;
				Math::Vector3d T0T2 = tv2.first - tv0.first;
				Math::Vector3d T = tv0.first + triangleAlpha * T0T1 + triangleBeta * T0T2;
				Math::Vector3d Tn = T0T1.cross(T0T2).normalized();
				double penetrationAtT0 = (S - T).dot(Tn);
				std::cout << "[t=0; " << penetrationAtT0 << "]";
			}

			{
				Math::Vector3d S = Math::interpolate(
					Math::interpolate(sv0.first, sv0.second, earliestTimeOfImpact),
					Math::interpolate(sv1.first, sv1.second, earliestTimeOfImpact), segmentAlpha);

				auto T0 = Math::interpolate(tv0.first, tv0.second, earliestTimeOfImpact);
				auto T1 = Math::interpolate(tv1.first, tv1.second, earliestTimeOfImpact);
				auto T2 = Math::interpolate(tv2.first, tv2.second, earliestTimeOfImpact);
				Math::Vector3d T0T1 = T1 - T0;
				Math::Vector3d T0T2 = T2 - T0;
				Math::Vector3d T = T0 + triangleAlpha * T0T1 + triangleBeta * T0T2;
				Math::Vector3d Tn = T0T1.cross(T0T2).normalized();
				double penetrationAtTimeOfImpact = (S - T).dot(Tn);
				std::cout << "[t="<<earliestTimeOfImpact<<"; " << penetrationAtTimeOfImpact  << "]";
			}

			Math::Vector3d T, Tn;
			double penentrationDepthAtT1;
			{
				Math::Vector3d S = Math::interpolate(sv0.second, sv1.second, segmentAlpha);
				Math::Vector3d T0T1 = tv1.second - tv0.second;
				Math::Vector3d T0T2 = tv2.second - tv0.second;
				T = tv0.second + triangleAlpha * T0T1 + triangleBeta * T0T2;
				Tn = T0T1.cross(T0T2).normalized();
				penentrationDepthAtT1 = (S - T).dot(Tn);
				std::cout << "[t=1; " << penentrationDepthAtT1 << "]" << std::endl;
			}


			Math::Vector segmentBaryCoord(2);
			segmentBaryCoord << 1.0 - segmentAlpha, segmentAlpha;
			DataStructures::IndexedLocalCoordinate localCoordinateSegment(edgeId, segmentBaryCoord);
			DataStructures::Location locationSegment(localCoordinateSegment, Location::ELEMENT);

			Math::Vector triangleBaryCoord(3);
			triangleBaryCoord << 1.0 - triangleAlpha - triangleBeta, triangleAlpha, triangleBeta;
			DataStructures::IndexedLocalCoordinate localCoordinateTriangle(triangleId, triangleBaryCoord);
			// The location related to the TriangleMesh can carry a TRIANGLE information (part of a Mass-Spring with deformable triangulation for collision)
			DataStructures::Location locationTriangle(localCoordinateTriangle, Location::TRIANGLE);
			// The location related to the TriangleMesh can carry an ELEMENT information (part of an Fem2D for example)
			locationTriangle.elementMeshLocalCoordinate = locationTriangle.triangleMeshLocalCoordinate;
			// The location related to the TriangleMesh can carry a RIGID LOCAL POSITION information (part of a rigid body)
			locationTriangle.rigidLocalPosition = pose2AtTime1.inverse() * T;

			auto contact = std::make_shared<Contact>(
				COLLISION_DETECTION_TYPE_CONTINUOUS,
				penentrationDepthAtT1,
				earliestTimeOfImpact,
				T,
				Tn,
				std::make_pair(locationSegment, locationTriangle));

			if (!isThisContactADuplicate(contact, contacts))
			{
				contacts.push_back(contact);
			}
		}
	}

	bool isContactingAtT0 = false;
	for (const auto& contact : contacts)
	{
		if (contact->time == 0.0)
		{
			isContactingAtT0 = true;
			break;
		}
	}
	if (isContactingAtT0)
	{
		std::cout << "Contacting at t=0" << std::endl;
	}

	//std::stringstream ss;
	//if (segmentMeshShapeMoved)
	//{
	//	ss << "SegmentMeshShape did not move; ";
	//}
	//if (meshShapeMoved)
	//{
	//	ss << "MeshShape did not move; ";
	//}
	//if (segmentMeshShapeMoved || meshShapeMoved)
	//{
	//	ss << std::endl;
	//}
	//std::cout << ss.str();

	return contacts;
}


}; // namespace Collision
}; // namespace SurgSim
