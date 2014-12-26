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

#include <gtest/gtest.h>

#include <memory>
#include <array>

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem3DElementCube.h"

using SurgSim::Math::Matrix;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::Fem3DElementCube;

namespace
{
/// Epsilon used in this unit test, resulting from a trial and error test.
const double epsilon = 3.6e-9;
};

class MockFem3DElementCube : public Fem3DElementCube
{
public:
	explicit MockFem3DElementCube(std::array<size_t, 8> nodeIds) : Fem3DElementCube(nodeIds)
	{
	}

	double getRestVolume() const
	{
		return m_restVolume;
	}

	double evaluateN(int i, double epsilon, double eta, double mu) const
	{
		return shapeFunction(i, epsilon, eta, mu);
	}

	double evaluatedNidEpsilon(int i, double epsilon, double eta, double mu) const
	{
		return dShapeFunctiondepsilon(i, epsilon, eta, mu);
	}

	double evaluatedNidEta(int i, double epsilon, double eta, double mu) const
	{
		return dShapeFunctiondeta(i, epsilon, eta, mu);
	}

	double evaluatedNidMu(int i, double epsilon, double eta, double mu) const
	{
		return dShapeFunctiondmu(i, epsilon, eta, mu);
	}

	const Eigen::Matrix<double, 24, 1>& getInitialPosition() const
	{
		return m_elementRestPosition;
	}
};

class Fem3DElementCubeTests : public ::testing::Test
{
public:
	std::array<size_t, 8> m_nodeIds;
	SurgSim::Math::OdeState m_restState;
	double m_expectedVolume;
	Eigen::Matrix<double, 24, 1> m_expectedX0;
	double m_rho, m_E, m_nu;
	SurgSim::Math::Matrix m_expectedMassMatrix, m_expectedDampingMatrix, m_expectedStiffnessMatrix;
	SurgSim::Math::Vector m_vectorOnes;

	std::shared_ptr<MockFem3DElementCube> getCubeElement(const std::array<size_t, 8>& nodeIds)
	{
		std::shared_ptr<MockFem3DElementCube> element;

		element = std::make_shared<MockFem3DElementCube>(nodeIds);
		element->setYoungModulus(m_E);
		element->setPoissonRatio(m_nu);
		element->setMassDensity(m_rho);

		return element;
	}

	void computeExpectedStiffnessMatrix(std::vector<size_t> nodeIdsVectorForm)
	{
		using SurgSim::Math::getSubMatrix;
		using SurgSim::Math::addSubMatrix;

		Eigen::Matrix<double, 24, 24> K;
		K.setZero();
		{
			// Expected stiffness matrix given in
			// "Physically-Based Simulation of Objects Represented by Surface Meshes"
			// Muller, Techner, Gross, CGI 2004
			// NOTE a bug in the paper, the sub matrix K44 should have a diagonal of 'd'
			double h = 1.0;
			double a = h * m_E * (1.0 - m_nu) / ((1.0 + m_nu) * (1.0 - 2.0 * m_nu));
			double b = h * m_E * (m_nu) / ((1.0 + m_nu) * (1.0 - 2.0 * m_nu));
			double c = h * m_E / (2.0 * (1.0 + m_nu));

			double d = (a + 2.0 * c) / 9.0;
			double e = (b + c) / 12.0;
			double n = -e;
			// Fill up the diagonal sub-matrices (3x3)
			getSubMatrix(K, 0, 0, 3, 3).setConstant(e);
			getSubMatrix(K, 0, 0, 3, 3).diagonal().setConstant(d);

			getSubMatrix(K, 1, 1, 3, 3).setConstant(n);
			getSubMatrix(K, 1, 1, 3, 3).diagonal().setConstant(d);
			getSubMatrix(K, 1, 1, 3, 3)(1,2) = e;
			getSubMatrix(K, 1, 1, 3, 3)(2,1) = e;

			getSubMatrix(K, 2, 2, 3, 3).setConstant(n);
			getSubMatrix(K, 2, 2, 3, 3).diagonal().setConstant(d);
			getSubMatrix(K, 2, 2, 3, 3)(0,1) = e;
			getSubMatrix(K, 2, 2, 3, 3)(1,0) = e;

			getSubMatrix(K, 3, 3, 3, 3).setConstant(n);
			getSubMatrix(K, 3, 3, 3, 3).diagonal().setConstant(d);
			getSubMatrix(K, 3, 3, 3, 3)(0,2) = e;
			getSubMatrix(K, 3, 3, 3, 3)(2,0) = e;

			getSubMatrix(K, 4, 4, 3, 3).setConstant(n);
			getSubMatrix(K, 4, 4, 3, 3).diagonal().setConstant(d);
			//getSubMatrix(K, 4, 4, 3, 3)(0,0) = e; // BUG IN THE PAPER !!!
			getSubMatrix(K, 4, 4, 3, 3)(0,1) = e;
			getSubMatrix(K, 4, 4, 3, 3)(1,0) = e;

			getSubMatrix(K, 5, 5, 3, 3).setConstant(n);
			getSubMatrix(K, 5, 5, 3, 3).diagonal().setConstant(d);
			getSubMatrix(K, 5, 5, 3, 3)(0,2) = e;
			getSubMatrix(K, 5, 5, 3, 3)(2,0) = e;

			getSubMatrix(K, 6, 6, 3, 3).setConstant(e);
			getSubMatrix(K, 6, 6, 3, 3).diagonal().setConstant(d);

			getSubMatrix(K, 7, 7, 3, 3).setConstant(n);
			getSubMatrix(K, 7, 7, 3, 3).diagonal().setConstant(d);
			getSubMatrix(K, 7, 7, 3, 3)(1,2) = e;
			getSubMatrix(K, 7, 7, 3, 3)(2,1) = e;

			// Edges
			{
				double d1 = (-a + c) / 9.0;
				double d2 = (a - c) / 18.0;
				double e1 = (b - c) / 12.0;
				double e2 = (b + c) / 24.0;
				double n1 = -e1;
				double n2 = -e2;

				// Edge in x-direction
				getSubMatrix(K, 0, 1, 3, 3)(0, 0) = d1;
				getSubMatrix(K, 0, 1, 3, 3)(0, 1) = e1;
				getSubMatrix(K, 0, 1, 3, 3)(0, 2) = e1;
				getSubMatrix(K, 0, 1, 3, 3)(1, 0) = n1;
				getSubMatrix(K, 0, 1, 3, 3)(1, 1) = d2;
				getSubMatrix(K, 0, 1, 3, 3)(1, 2) = e2;
				getSubMatrix(K, 0, 1, 3, 3)(2, 0) = n1;
				getSubMatrix(K, 0, 1, 3, 3)(2, 1) = e2;
				getSubMatrix(K, 0, 1, 3, 3)(2, 2) = d2;

				getSubMatrix(K, 2, 3, 3, 3) = getSubMatrix(K, 0, 1, 3, 3);
				getSubMatrix(K, 2, 3, 3, 3)(0, 2) = n1;
				getSubMatrix(K, 2, 3, 3, 3)(1, 2) = n2;
				getSubMatrix(K, 2, 3, 3, 3)(2, 0) = e1;
				getSubMatrix(K, 2, 3, 3, 3)(2, 1) = n2;

				getSubMatrix(K, 4, 5, 3, 3) = getSubMatrix(K, 2, 3, 3, 3);

				getSubMatrix(K, 6, 7, 3, 3) = getSubMatrix(K, 0, 1, 3, 3);

				// Edge in y-direction
				getSubMatrix(K, 0, 3, 3, 3)(0, 0) = d2;
				getSubMatrix(K, 0, 3, 3, 3)(0, 1) = n1;
				getSubMatrix(K, 0, 3, 3, 3)(0, 2) = e2;
				getSubMatrix(K, 0, 3, 3, 3)(1, 0) = e1;
				getSubMatrix(K, 0, 3, 3, 3)(1, 1) = d1;
				getSubMatrix(K, 0, 3, 3, 3)(1, 2) = e1;
				getSubMatrix(K, 0, 3, 3, 3)(2, 0) = e2;
				getSubMatrix(K, 0, 3, 3, 3)(2, 1) = n1;
				getSubMatrix(K, 0, 3, 3, 3)(2, 2) = d2;

				getSubMatrix(K, 1, 2, 3, 3) = getSubMatrix(K, 0, 3, 3, 3);
				getSubMatrix(K, 1, 2, 3, 3)(0, 1) = e1;
				getSubMatrix(K, 1, 2, 3, 3)(0, 2) = n2;
				getSubMatrix(K, 1, 2, 3, 3)(1, 0) = n1;
				getSubMatrix(K, 1, 2, 3, 3)(2, 0) = n2;

				getSubMatrix(K, 4, 7, 3, 3) = getSubMatrix(K, 1, 2, 3, 3);
				getSubMatrix(K, 4, 7, 3, 3)(0, 1) = n1;
				getSubMatrix(K, 4, 7, 3, 3)(1, 0) = e1;
				getSubMatrix(K, 4, 7, 3, 3)(1, 2) = n1;
				getSubMatrix(K, 4, 7, 3, 3)(2, 1) = e1;

				getSubMatrix(K, 5, 6, 3, 3) = getSubMatrix(K, 0, 3, 3, 3);
				getSubMatrix(K, 5, 6, 3, 3)(0, 1) = e1;
				getSubMatrix(K, 5, 6, 3, 3)(1, 0) = n1;
				getSubMatrix(K, 5, 6, 3, 3)(1, 2) = n1;
				getSubMatrix(K, 5, 6, 3, 3)(2, 1) = e1;

				// Edge in z-direction
				getSubMatrix(K, 0, 4, 3, 3)(0, 0) = d2;
				getSubMatrix(K, 0, 4, 3, 3)(0, 1) = e2;
				getSubMatrix(K, 0, 4, 3, 3)(0, 2) = n1;
				getSubMatrix(K, 0, 4, 3, 3)(1, 0) = e2;
				getSubMatrix(K, 0, 4, 3, 3)(1, 1) = d2;
				getSubMatrix(K, 0, 4, 3, 3)(1, 2) = n1;
				getSubMatrix(K, 0, 4, 3, 3)(2, 0) = e1;
				getSubMatrix(K, 0, 4, 3, 3)(2, 1) = e1;
				getSubMatrix(K, 0, 4, 3, 3)(2, 2) = d1;

				getSubMatrix(K, 1, 5, 3, 3) = getSubMatrix(K, 0, 4, 3, 3);
				getSubMatrix(K, 1, 5, 3, 3)(0, 1) = n2;
				getSubMatrix(K, 1, 5, 3, 3)(0, 2) = e1;
				getSubMatrix(K, 1, 5, 3, 3)(1, 0) = n2;
				getSubMatrix(K, 1, 5, 3, 3)(2, 0) = n1;

				getSubMatrix(K, 2, 6, 3, 3) = getSubMatrix(K, 0, 4, 3, 3);
				getSubMatrix(K, 2, 6, 3, 3)(0, 2) = e1;
				getSubMatrix(K, 2, 6, 3, 3)(1, 2) = e1;
				getSubMatrix(K, 2, 6, 3, 3)(2, 0) = n1;
				getSubMatrix(K, 2, 6, 3, 3)(2, 1) = n1;

				getSubMatrix(K, 3, 7, 3, 3) = getSubMatrix(K, 0, 4, 3, 3);
				getSubMatrix(K, 3, 7, 3, 3)(0, 1) = n2;
				getSubMatrix(K, 3, 7, 3, 3)(1, 0) = n2;
				getSubMatrix(K, 3, 7, 3, 3)(1, 2) = e1;
				getSubMatrix(K, 3, 7, 3, 3)(2, 1) = n1;
			}

			// Faces diagonals
			{
				double d1 = (-2.0 * a - c) / 36.0;
				double d2 = (a - 4.0 * c) / 36.0;
				double e1 = (b + c) / 12.0;
				double e2 = (b - c) / 24.0;
				double n1 = -e1;
				double n2 = -e2;

				getSubMatrix(K, 0, 5, 3, 3)(0, 0) = d1;
				getSubMatrix(K, 0, 5, 3, 3)(0, 1) = e2;
				getSubMatrix(K, 0, 5, 3, 3)(0, 2) = n1;
				getSubMatrix(K, 0, 5, 3, 3)(1, 0) = n2;
				getSubMatrix(K, 0, 5, 3, 3)(1, 1) = d2;
				getSubMatrix(K, 0, 5, 3, 3)(1, 2) = n2;
				getSubMatrix(K, 0, 5, 3, 3)(2, 0) = n1;
				getSubMatrix(K, 0, 5, 3, 3)(2, 1) = e2;
				getSubMatrix(K, 0, 5, 3, 3)(2, 2) = d1;

				getSubMatrix(K, 1, 4, 3, 3) = getSubMatrix(K, 0, 5, 3, 3);
				getSubMatrix(K, 1, 4, 3, 3)(0, 1) = n2;
				getSubMatrix(K, 1, 4, 3, 3)(0, 2) = e1;
				getSubMatrix(K, 1, 4, 3, 3)(1, 0) = e2;
				getSubMatrix(K, 1, 4, 3, 3)(2, 0) = e1;

				getSubMatrix(K, 2, 7, 3, 3) = getSubMatrix(K, 0, 5, 3, 3);
				getSubMatrix(K, 2, 7, 3, 3)(0, 2) = e1;
				getSubMatrix(K, 2, 7, 3, 3)(1, 2) = e2;
				getSubMatrix(K, 2, 7, 3, 3)(2, 0) = e1;
				getSubMatrix(K, 2, 7, 3, 3)(2, 1) = n2;

				getSubMatrix(K, 3, 6, 3, 3) = getSubMatrix(K, 0, 5, 3, 3);
				getSubMatrix(K, 3, 6, 3, 3)(0, 1) = n2;
				getSubMatrix(K, 3, 6, 3, 3)(1, 0) = e2;
				getSubMatrix(K, 3, 6, 3, 3)(1, 2) = e2;
				getSubMatrix(K, 3, 6, 3, 3)(2, 1) = n2;

				getSubMatrix(K, 1, 6, 3, 3)(0, 0) = d2;
				getSubMatrix(K, 1, 6, 3, 3)(0, 1) = e2;
				getSubMatrix(K, 1, 6, 3, 3)(0, 2) = e2;
				getSubMatrix(K, 1, 6, 3, 3)(1, 0) = n2;
				getSubMatrix(K, 1, 6, 3, 3)(1, 1) = d1;
				getSubMatrix(K, 1, 6, 3, 3)(1, 2) = n1;
				getSubMatrix(K, 1, 6, 3, 3)(2, 0) = n2;
				getSubMatrix(K, 1, 6, 3, 3)(2, 1) = n1;
				getSubMatrix(K, 1, 6, 3, 3)(2, 2) = d1;

				getSubMatrix(K, 2, 5, 3, 3) = getSubMatrix(K, 1, 6, 3, 3);
				getSubMatrix(K, 2, 5, 3, 3)(0, 1) = n2;
				getSubMatrix(K, 2, 5, 3, 3)(1, 0) = e2;
				getSubMatrix(K, 2, 5, 3, 3)(1, 2) = e1;
				getSubMatrix(K, 2, 5, 3, 3)(2, 1) = e1;

				getSubMatrix(K, 0, 7, 3, 3) = getSubMatrix(K, 1, 6, 3, 3);
				getSubMatrix(K, 0, 7, 3, 3)(0, 1) = n2;
				getSubMatrix(K, 0, 7, 3, 3)(0, 2) = n2;
				getSubMatrix(K, 0, 7, 3, 3)(1, 0) = e2;
				getSubMatrix(K, 0, 7, 3, 3)(2, 0) = e2;

				getSubMatrix(K, 3, 4, 3, 3) = getSubMatrix(K, 1, 6, 3, 3);
				getSubMatrix(K, 3, 4, 3, 3)(0, 2) = n2;
				getSubMatrix(K, 3, 4, 3, 3)(1, 2) = e1;
				getSubMatrix(K, 3, 4, 3, 3)(2, 0) = e2;
				getSubMatrix(K, 3, 4, 3, 3)(2, 1) = e1;

				getSubMatrix(K, 0, 2, 3, 3)(0, 0) = d1;
				getSubMatrix(K, 0, 2, 3, 3)(0, 1) = n1;
				getSubMatrix(K, 0, 2, 3, 3)(0, 2) = e2;
				getSubMatrix(K, 0, 2, 3, 3)(1, 0) = n1;
				getSubMatrix(K, 0, 2, 3, 3)(1, 1) = d1;
				getSubMatrix(K, 0, 2, 3, 3)(1, 2) = e2;
				getSubMatrix(K, 0, 2, 3, 3)(2, 0) = n2;
				getSubMatrix(K, 0, 2, 3, 3)(2, 1) = n2;
				getSubMatrix(K, 0, 2, 3, 3)(2, 2) = d2;

				getSubMatrix(K, 1, 3, 3, 3) = getSubMatrix(K, 0, 2, 3, 3);
				getSubMatrix(K, 1, 3, 3, 3)(0, 1) = e1;
				getSubMatrix(K, 1, 3, 3, 3)(0, 2) = n2;
				getSubMatrix(K, 1, 3, 3, 3)(1, 0) = e1;
				getSubMatrix(K, 1, 3, 3, 3)(2, 0) = e2;

				getSubMatrix(K, 4, 6, 3, 3) = getSubMatrix(K, 0, 2, 3, 3);
				getSubMatrix(K, 4, 6, 3, 3)(0, 2) = n2;
				getSubMatrix(K, 4, 6, 3, 3)(1, 2) = n2;
				getSubMatrix(K, 4, 6, 3, 3)(2, 0) = e2;
				getSubMatrix(K, 4, 6, 3, 3)(2, 1) = e2;

				getSubMatrix(K, 5, 7, 3, 3) = getSubMatrix(K, 0, 2, 3, 3);
				getSubMatrix(K, 5, 7, 3, 3)(0, 1) = e1;
				getSubMatrix(K, 5, 7, 3, 3)(1, 0) = e1;
				getSubMatrix(K, 5, 7, 3, 3)(1, 2) = n2;
				getSubMatrix(K, 5, 7, 3, 3)(2, 1) = e2;
			}

			// Cube diagonals
			{
				double d = (-a - 2.0 * c) / 36.0;
				double e = (b + c) / 24.0;
				double n = -e;

				getSubMatrix(K, 0, 6, 3, 3).setConstant(n);
				getSubMatrix(K, 0, 6, 3, 3).diagonal().setConstant(d);

				getSubMatrix(K, 1, 7, 3, 3).setConstant(e);
				getSubMatrix(K, 1, 7, 3, 3).diagonal().setConstant(d);
				getSubMatrix(K, 1, 7, 3, 3)(1, 2) = n;
				getSubMatrix(K, 1, 7, 3, 3)(2, 1) = n;

				getSubMatrix(K, 2, 4, 3, 3).setConstant(e);
				getSubMatrix(K, 2, 4, 3, 3).diagonal().setConstant(d);
				getSubMatrix(K, 2, 4, 3, 3)(0, 1) = n;
				getSubMatrix(K, 2, 4, 3, 3)(1, 0) = n;

				getSubMatrix(K, 3, 5, 3, 3).setConstant(e);
				getSubMatrix(K, 3, 5, 3, 3).diagonal().setConstant(d);
				getSubMatrix(K, 3, 5, 3, 3)(0, 2) = n;
				getSubMatrix(K, 3, 5, 3, 3)(2, 0) = n;
			}

			// Use symmetry to complete the triangular inferior part of K
			K.triangularView<Eigen::StrictlyLower>().setZero();
			K += K.triangularView<Eigen::StrictlyUpper>().adjoint();
		}
		addSubMatrix(K, nodeIdsVectorForm, 3 , &m_expectedStiffnessMatrix);
	}

	void computeExpectedMassMatrix(std::vector<size_t> nodeIdsVectorForm)
	{
		using SurgSim::Math::addSubMatrix;

		Eigen::Matrix<double, 24, 24> M;
		M.setZero();

		// "Physically-Based Simulation of Objects Represented by Surface Meshes"
		// Muller, Techner, Gross, CGI 2004
		// Given the shape functions they defined on Appendix A for the cube we are testing
		// We can derive the mass matrix M = \int_V rho N^T.N dV
		// N being a matrix (3x24) of shape functions
		// cf documentation

		double a = 1.0 / 27.0;
		double b = a / 2.0;
		double c = a / 4.0;
		double d = a / 8.0;

		M.diagonal().setConstant(a);

		M.block(0, 3, 21, 21).diagonal().setConstant(b);
		M.block(3*3, 3*4, 3, 3).diagonal().setConstant(c); // block (3, 4)

		M.block(0, 6, 18, 18).diagonal().setConstant(c);
		M.block(3*2, 3*4, 6, 6).diagonal().setConstant(d); // block (2, 4) and block (3, 5)

		M.block(0, 9, 15, 15).diagonal().setConstant(c);
		M.block(3*0, 3*3, 3, 3).diagonal().setConstant(b); // block (0, 3)
		M.block(3*4, 3*7, 3, 3).diagonal().setConstant(b); // block (4, 7)

		M.block(0, 12, 12, 12).diagonal().setConstant(b);

		M.block(0, 15, 9, 9).diagonal().setConstant(c);

		M.block(0, 18, 6, 6).diagonal().setConstant(d);

		M.block(0, 21, 3, 3).diagonal().setConstant(c);

		// Symmetry
		for (size_t row = 0; row < 24; ++row)
		{
			for (size_t col = row+1; col < 24; ++col)
			{
				M(col, row) = M(row, col);
			}
		}

		M *= m_rho;
		addSubMatrix(M, nodeIdsVectorForm, 3 , &m_expectedMassMatrix);
	}

	void SetUp() override
	{
		using SurgSim::Math::getSubVector;
		using SurgSim::Math::getSubMatrix;
		using SurgSim::Math::addSubMatrix;

		m_restState.setNumDof(3, 8);
		Vector& x0 = m_restState.getPositions();

		// Cube is aligned with the axis (X,Y,Z), centered on (0.0, 0.0, 0.0), of size 1
		//       2*-----------*3
		//       /           /|
		//    6*-----------*7 |      ^ y
		//     |           |  |      |
		//     |  0        |  *1     *->x
		//     |           | /      /
		//    4*-----------*5       z
		getSubVector(x0, 0, 3) = Vector3d(-0.5,-0.5,-0.5);
		getSubVector(x0, 1, 3) = Vector3d( 0.5,-0.5,-0.5);
		getSubVector(x0, 2, 3) = Vector3d(-0.5, 0.5,-0.5);
		getSubVector(x0, 3, 3) = Vector3d( 0.5, 0.5,-0.5);
		getSubVector(x0, 4, 3) = Vector3d(-0.5,-0.5, 0.5);
		getSubVector(x0, 5, 3) = Vector3d( 0.5,-0.5, 0.5);
		getSubVector(x0, 6, 3) = Vector3d(-0.5, 0.5, 0.5);
		getSubVector(x0, 7, 3) = Vector3d( 0.5, 0.5, 0.5);

		// Ordering following the description in
		// "Physically-Based Simulation of Objects Represented by Surface Meshes"
		// Muller, Techner, Gross, CGI 2004
		std::array<size_t, 8> tmpNodeIds = {{0, 1, 3, 2, 4, 5, 7, 6}};
		m_nodeIds = tmpNodeIds;

		// Useful for assembly helper function
		std::vector<size_t> nodeIdsVectorForm(tmpNodeIds.begin(), tmpNodeIds.end());

		// Build the expected x0 vector
		for (size_t i = 0; i < 8; i++)
		{
			getSubVector(m_expectedX0, i, 3) = getSubVector(x0, m_nodeIds[i], 3);
		}

		// The cube has a size of 1, so its volume is 1m^3
		m_expectedVolume = 1.0;

		m_rho = 1000.0;
		m_E = 1e6;
		m_nu = 0.45;

		m_expectedMassMatrix.resize(3*8, 3*8);
		m_expectedMassMatrix.setZero();
		m_expectedDampingMatrix.resize(3*8, 3*8);
		m_expectedDampingMatrix.setZero();
		m_expectedStiffnessMatrix.resize(3*8, 3*8);
		m_expectedStiffnessMatrix.setZero();
		m_vectorOnes.resize(3*8);
		m_vectorOnes.setConstant(1.0);

		computeExpectedMassMatrix(nodeIdsVectorForm);
		m_expectedDampingMatrix.setZero();
		computeExpectedStiffnessMatrix(nodeIdsVectorForm);
	}

	// This method tests all node permutations for both face definition (2 groups of 4 indices)
	// keeping their ordering intact (CW or CCW)
	void testNodeOrderingAllPermutations(const SurgSim::Math::OdeState& m_restState,
		size_t id0, size_t id1, size_t id2, size_t id3,
		size_t id4, size_t id5, size_t id6, size_t id7,
		bool expectThrow)
	{
		std::array<size_t, 4> face1 = {{id0, id1, id2, id3}};
		std::array<size_t, 4> face2 = {{id4, id5, id6, id7}};

		// Shuffle the faces to create all the possible permutations
		for (size_t face1Permutation = 0; face1Permutation < 4; face1Permutation++)
		{
			for (size_t face2Permutation = 0; face2Permutation < 4; face2Permutation++)
			{
				std::array<size_t, 8> ids;
				for (size_t index = 0; index < 4; index++)
				{
					ids[    index] = face1[(index + face1Permutation) % 4];
					ids[4 + index] = face2[(index + face2Permutation) % 4];
				}

				// Test this permutation
				if (expectThrow)
				{
					EXPECT_ANY_THROW({auto cube = getCubeElement(ids); cube->initialize(m_restState);});
				}
				else
				{
					EXPECT_NO_THROW({auto cube = getCubeElement(ids); cube->initialize(m_restState);});
				}
			}
		}
	}
};

extern void testSize(const Vector& v, int expectedSize);
extern void testSize(const Matrix& m, int expectedRows, int expectedCols);

TEST_F(Fem3DElementCubeTests, ConstructorTest)
{
	ASSERT_NO_THROW({MockFem3DElementCube cube(m_nodeIds);});
	ASSERT_NO_THROW({MockFem3DElementCube* cube = new MockFem3DElementCube(m_nodeIds); delete cube;});
	ASSERT_NO_THROW({std::shared_ptr<MockFem3DElementCube> cube =
		std::make_shared<MockFem3DElementCube>(m_nodeIds);});
}

TEST_F(Fem3DElementCubeTests, InitializeTest)
{
	{
		SCOPED_TRACE("Invalid node ids");

		std::array<size_t, 8> invalidNodeIds = {{0, 1, 2, 3, 4, 10, 9, 8}};
		ASSERT_NO_THROW({auto cube = getCubeElement(invalidNodeIds);});
		auto cube = getCubeElement(invalidNodeIds);
		ASSERT_THROW(cube->initialize(m_restState), SurgSim::Framework::AssertionFailure);
	}

	{
		SCOPED_TRACE("Valid node ids");

		ASSERT_NO_THROW({auto cube = getCubeElement(m_nodeIds);});
		auto cube = getCubeElement(m_nodeIds);
		ASSERT_NO_THROW(cube->initialize(m_restState));
	}
}

TEST_F(Fem3DElementCubeTests, NodeIdsTest)
{
	Fem3DElementCube cube(m_nodeIds);
	EXPECT_EQ(8u, cube.getNumNodes());
	EXPECT_EQ(8u, cube.getNodeIds().size());
	for (int i = 0; i < 8; i++)
	{
		EXPECT_EQ(m_nodeIds[i], cube.getNodeId(i));
		EXPECT_EQ(m_nodeIds[i], cube.getNodeIds()[i]);
	}
}

TEST_F(Fem3DElementCubeTests, VolumeTest)
{
	{
		SCOPED_TRACE("Volume valid and positive");

		auto cube = getCubeElement(m_nodeIds);
		cube->initialize(m_restState); // rest volume is computed by the initialize method
		EXPECT_NEAR(cube->getRestVolume(), m_expectedVolume, 1e-10);
		EXPECT_NEAR(cube->getVolume(m_restState), m_expectedVolume, 1e-10);
	}

	{
		SCOPED_TRACE("Volume valid but negative");

		std::array<size_t, 8> nodeIds = {{0, 1, 3, 2, 4, 6, 7, 5}};
		auto cube = getCubeElement(nodeIds);
		ASSERT_THROW(cube->getVolume(m_restState), SurgSim::Framework::AssertionFailure);
	}

	{
		SCOPED_TRACE("Volume invalid, degenerated cube");

		// Copy the 1st 4 points over the 4 following points, so the cube degenerate to a square
		m_restState.getPositions().segment<3 * 4>(12) = m_restState.getPositions().segment<3 * 4>(0);
		auto cube = getCubeElement(m_nodeIds);
		ASSERT_THROW(cube->getVolume(m_restState), SurgSim::Framework::AssertionFailure);
	}
}

TEST_F(Fem3DElementCubeTests, NodeOrderingTest)
{
	// Any definition starting with a 1st face defined CW
	// followed by the opposite face defined CCW is valid.

	// Front face CW, opposite face CCW
	testNodeOrderingAllPermutations(m_restState, 6, 7, 5, 4, 2, 3, 1, 0, false);
	// Front face CW, opposite face CW
	testNodeOrderingAllPermutations(m_restState, 6, 7, 5, 4, 2, 0, 1, 3, true);
	// Front face CCW, opposite face CCW
	testNodeOrderingAllPermutations(m_restState, 6, 4, 5, 7, 2, 3, 1, 0, true);
	// Front face CCW, opposite face CW
	testNodeOrderingAllPermutations(m_restState, 6, 4, 5, 7, 2, 0, 1, 3, true);

	// Back face CW, opposite face CCW
	testNodeOrderingAllPermutations(m_restState, 0, 1, 3, 2, 4, 5, 7, 6, false);
	// Back face CW, opposite face CW
	testNodeOrderingAllPermutations(m_restState, 0, 1, 3, 2, 4, 6, 7, 5, true);
	// Back face CCW, opposite face CCW
	testNodeOrderingAllPermutations(m_restState, 0, 2, 3, 1, 4, 5, 7, 6, true);
	// Back face CCW, opposite face CW
	testNodeOrderingAllPermutations(m_restState, 0, 2, 3, 1, 4, 6, 7, 5, true);

	// Top face CW, opposite face CCW
	testNodeOrderingAllPermutations(m_restState, 2, 3, 7, 6, 0, 1, 5, 4, false);
	// Top face CW, opposite face CW
	testNodeOrderingAllPermutations(m_restState, 2, 3, 7, 6, 0, 4, 5, 1, true);
	// Top face CCW, opposite face CCW
	testNodeOrderingAllPermutations(m_restState, 2, 6, 7, 3, 0, 1, 5, 4, true);
	// Top face CCW, opposite face CW
	testNodeOrderingAllPermutations(m_restState, 2, 6, 7, 3, 0, 4, 5, 1, true);

	// Bottom face CW, opposite face CCW
	testNodeOrderingAllPermutations(m_restState, 0, 4, 5, 1, 6, 7, 3, 2, false);
	// Bottom face CW, opposite face CW
	testNodeOrderingAllPermutations(m_restState, 0, 4, 5, 1, 6, 2, 3, 7, true);
	// Bottom face CCW, opposite face CCW
	testNodeOrderingAllPermutations(m_restState, 0, 1, 5, 4, 6, 7, 3, 2, true);
	// Bottom face CCW, opposite face CW
	testNodeOrderingAllPermutations(m_restState, 0, 1, 5, 4, 6, 2, 3, 7, true);

	// Right face CW, opposite face CCW
	testNodeOrderingAllPermutations(m_restState, 3, 1, 5, 7, 2, 0, 4, 6, false);
	// Right face CW, opposite face CW
	testNodeOrderingAllPermutations(m_restState, 3, 1, 5, 7, 2, 6, 4, 0, true);
	// Right face CCW, opposite face CCW
	testNodeOrderingAllPermutations(m_restState, 3, 7, 5, 1, 2, 0, 4, 6, true);
	// Right face CCW, opposite face CW
	testNodeOrderingAllPermutations(m_restState, 3, 7, 5, 1, 2, 6, 4, 0, true);

	// Left face CW, opposite face CCW
	testNodeOrderingAllPermutations(m_restState, 0, 2, 6, 4, 1, 3, 7, 5, false);
	// Left face CW, opposite face CW
	testNodeOrderingAllPermutations(m_restState, 0, 2, 6, 4, 1, 5, 7, 3, true);
	// Left face CCW, opposite face CCW
	testNodeOrderingAllPermutations(m_restState, 0, 4, 6, 2, 1, 3, 7, 5, true);
	// Left face CCW, opposite face CW
	testNodeOrderingAllPermutations(m_restState, 0, 4, 6, 2, 1, 5, 7, 3, true);
}

TEST_F(Fem3DElementCubeTests, ShapeFunctionsTest)
{
	using SurgSim::Math::getSubVector;

	auto cube = getCubeElement(m_nodeIds);
	cube->initialize(m_restState);

	EXPECT_TRUE(cube->getInitialPosition().isApprox(m_expectedX0)) <<
		"x0 = " << cube->getInitialPosition().transpose() << std::endl << "x0 expected = " << m_expectedX0.transpose();

	// We should have by construction:
	// { N0(p0) = 1    N1(p0)=N2(p0)=N3(p0)=0
	// { N1(p1) = 1    N1(p1)=N2(p1)=N3(p1)=0
	// { N2(p2) = 1    N1(p2)=N2(p2)=N3(p2)=0
	// { N3(p3) = 1    N1(p3)=N2(p3)=N3(p3)=0
	Vector3d p[8];
	for (size_t nodeId = 0; nodeId < 8; ++nodeId)
	{
		// retrieving the points from expectedX0 with indices 0..7
		// which is equivalent to
		// retrieving the points from m_restState with indices m_nodeIds[0]..m_nodeIds[7]
		p[nodeId] = getSubVector(m_expectedX0, nodeId, 3);
	}
	double Ni_p0[8], Ni_p1[8], Ni_p2[8], Ni_p3[8], Ni_p4[8], Ni_p5[8], Ni_p6[8], Ni_p7[8];
	for (int i = 0; i < 8; i++)
	{
		Ni_p0[i] = cube->evaluateN(i, -1.0, -1.0, -1.0);
		Ni_p1[i] = cube->evaluateN(i, +1.0, -1.0, -1.0);
		Ni_p2[i] = cube->evaluateN(i, +1.0, +1.0, -1.0);
		Ni_p3[i] = cube->evaluateN(i, -1.0, +1.0, -1.0);
		Ni_p4[i] = cube->evaluateN(i, -1.0, -1.0, +1.0);
		Ni_p5[i] = cube->evaluateN(i, +1.0, -1.0, +1.0);
		Ni_p6[i] = cube->evaluateN(i, +1.0, +1.0, +1.0);
		Ni_p7[i] = cube->evaluateN(i, -1.0, +1.0, +1.0);
	}
	EXPECT_NEAR(Ni_p0[0], 1.0, 1e-12);
	for (size_t i = 0; i < 8; ++i)
	{
		if (i == 0) continue;
		EXPECT_NEAR(Ni_p0[i], 0.0, 1e-12);
	}

	EXPECT_NEAR(Ni_p1[1], 1.0, 1e-12);
	for (size_t i = 0; i < 8; ++i)
	{
		if (i == 1) continue;
		EXPECT_NEAR(Ni_p1[i], 0.0, 1e-12);
	}

	EXPECT_NEAR(Ni_p2[2], 1.0, 1e-12);
	for (size_t i = 0; i < 8; ++i)
	{
		if (i == 2) continue;
		EXPECT_NEAR(Ni_p2[i], 0.0, 1e-12);
	}

	EXPECT_NEAR(Ni_p3[3], 1.0, 1e-12);
	for (size_t i = 0; i < 8; ++i)
	{
		if (i == 3) continue;
		EXPECT_NEAR(Ni_p3[i], 0.0, 1e-12);
	}

	EXPECT_NEAR(Ni_p4[4], 1.0, 1e-12);
	for (size_t i = 0; i < 8; ++i)
	{
		if (i == 4) continue;
		EXPECT_NEAR(Ni_p4[i], 0.0, 1e-12);
	}

	EXPECT_NEAR(Ni_p5[5], 1.0, 1e-12);
	for (size_t i = 0; i < 8; ++i)
	{
		if (i == 5) continue;
		EXPECT_NEAR(Ni_p5[i], 0.0, 1e-12);
	}

	EXPECT_NEAR(Ni_p6[6], 1.0, 1e-12);
	for (size_t i = 0; i < 8; ++i)
	{
		if (i == 6) continue;
		EXPECT_NEAR(Ni_p6[i], 0.0, 1e-12);
	}

	EXPECT_NEAR(Ni_p7[7], 1.0, 1e-12);
	for (size_t i = 0; i < 8; ++i)
	{
		if (i == 7) continue;
		EXPECT_NEAR(Ni_p7[i], 0.0, 1e-12);
	}

	// We should have the relation sum(Ni(x,y,z) = 1) for all points in the volume
	// We verify that relation by sampling the tetrahedron volume
	for (double epsilon = -1.0; epsilon <= 1.0; epsilon+=0.1)
	{
		for (double eta = -1.0; eta <= 1.0; eta+=0.1)
		{
			for (double mu = -1.0; mu <= 1.0; mu+=0.1)
			{
				double Ni_p[8];
				double sum = 0.0;
				for (int i = 0; i < 8; i++)
				{
					Ni_p[i] = cube->evaluateN(i, epsilon, eta, mu);
					sum += Ni_p[i];
				}
				EXPECT_NEAR(sum, 1.0, 1e-10) <<
					" for epsilon = " << epsilon << ", eta = " << eta << ", mu = " << mu << std::endl <<
					" N0(epsilon,eta,mu) = " << Ni_p[0] << " N1(epsilon,eta,mu) = " << Ni_p[1] <<
					" N2(epsilon,eta,mu) = " << Ni_p[2] << " N3(epsilon,eta,mu) = " << Ni_p[3] <<
					" N4(epsilon,eta,mu) = " << Ni_p[4] << " N5(epsilon,eta,mu) = " << Ni_p[5] <<
					" N6(epsilon,eta,mu) = " << Ni_p[6] << " N7(epsilon,eta,mu) = " << Ni_p[7];
			}
		}
	}
}

TEST_F(Fem3DElementCubeTests, CoordinateTests)
{
	auto cube = getCubeElement(m_nodeIds);
	cube->initialize(m_restState);

	{
		// Non-normalize node
		SurgSim::Math::Vector nodePositions(8);
		nodePositions <<  0.7, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		EXPECT_FALSE(cube->isValidCoordinate(nodePositions));

		// Node with point which is outside of the cube
		nodePositions <<  1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		EXPECT_FALSE(cube->isValidCoordinate(nodePositions));

		// Normal node
		nodePositions <<  0.5, 0.25, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0;
		EXPECT_TRUE(cube->isValidCoordinate(nodePositions));

	}

	{
		// Node with more than 8 coordinate points
		SurgSim::Math::Vector nodePositions(9);
		nodePositions <<  1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		EXPECT_FALSE(cube->isValidCoordinate(nodePositions));
	}

	{
		// Node with some coordinates less than 0 but greater than epsilon and
		// some greater than 1 and less than (1 + epsilon).
		SurgSim::Math::Vector nodePositions(8);
		nodePositions <<  -1e-11, 1.0 + 1e-11, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		EXPECT_TRUE(cube->isValidCoordinate(nodePositions));
	}

	// Test computeCartesianCoordinate.
	{
		// Compute central point of the cube
		SurgSim::Math::Vector nodePositions(8);
		nodePositions << 0.125, 0.125, 0.125, 0.125, 0.125, 0.125, 0.125, 0.125;
		EXPECT_TRUE(cube->isValidCoordinate(nodePositions));
		EXPECT_TRUE(SurgSim::Math::Vector3d(0.0, 0.0, 0.0).isApprox(
					cube->computeCartesianCoordinate(m_restState, nodePositions), epsilon));
	}

	{
		SurgSim::Math::Vector nodePositions(8);
		nodePositions << 0.01, 0.07, 0.11, 0.05, 0.0, 0.23, 0.13, 0.4;
		EXPECT_TRUE(cube->isValidCoordinate(nodePositions));
		EXPECT_TRUE(SurgSim::Math::Vector3d(0.04, 0.19, 0.26).isApprox(
					cube->computeCartesianCoordinate(m_restState, nodePositions), epsilon));
		// 0.01 * (-0.5,-0.5,-0.5) => (-0.005, -0.005, -0.005)
		// 0.07 * ( 0.5,-0.5,-0.5) => ( 0.035, -0.035, -0.035)
		// 0.11 * ( 0.5, 0.5,-0.5) => ( 0.055,  0.055, -0.055)
		// 0.05 * (-0.5, 0.5,-0.5) => (-0.025,  0.025, -0.025)
		// 0.0  * (-0.5,-0.5, 0.5) => (-0.00,  -0.00,   0.00)
		// 0.23 * ( 0.5,-0.5, 0.5) => ( 0.115, -0.115,  0.115)
		// 0.13 * ( 0.5, 0.5, 0.5) => ( 0.065,  0.065,  0.065)
		// 0.4  * (-0.5, 0.5, 0.5) => (-0.20,   0.20,   0.20)
		//                          = ( 0.04,   0.19,   0.26)
	}

	// Test computeNaturalCoordinate.
	EXPECT_THROW(cube->computeNaturalCoordinate(m_restState, SurgSim::Math::Vector3d(0.0, 0.0, 0.0)),
				 SurgSim::Framework::AssertionFailure);
}

TEST_F(Fem3DElementCubeTests, ForceAndMatricesTest)
{
	using SurgSim::Math::getSubVector;

	auto cube = getCubeElement(m_nodeIds);
	cube->initialize(m_restState);

	SurgSim::Math::Vector forceVector(3*8);
	SurgSim::Math::Matrix massMatrix(3*8, 3*8);
	SurgSim::Math::Matrix dampingMatrix(3*8, 3*8);
	SurgSim::Math::Matrix stiffnessMatrix(3*8, 3*8);

	forceVector.setZero();
	massMatrix.setZero();
	dampingMatrix.setZero();
	stiffnessMatrix.setZero();

	// No force should be produced when in rest state (x = x0) => F = K.(x-x0) = 0
	cube->addForce(m_restState, &forceVector);
	EXPECT_TRUE(forceVector.isZero());

	cube->addMass(m_restState, &massMatrix);
	EXPECT_TRUE(massMatrix.isApprox(m_expectedMassMatrix)) <<
		"Expected mass matrix :" << std::endl << m_expectedMassMatrix  << std::endl << std::endl <<
		"Mass matrix :"  << std::endl << massMatrix << std::endl << std::endl <<
		"Error on the mass matrix is "  << std::endl << m_expectedMassMatrix - massMatrix << std::endl;

	cube->addDamping(m_restState, &dampingMatrix);
	EXPECT_TRUE(dampingMatrix.isApprox(m_expectedDampingMatrix));

	cube->addStiffness(m_restState, &stiffnessMatrix);
	EXPECT_TRUE(stiffnessMatrix.isApprox(m_expectedStiffnessMatrix)) <<
		"Error on the stiffness matrix is " << std::endl << m_expectedStiffnessMatrix - stiffnessMatrix << std::endl;

	forceVector.setZero();
	massMatrix.setZero();
	dampingMatrix.setZero();
	stiffnessMatrix.setZero();

	cube->addFMDK(m_restState, &forceVector, &massMatrix, &dampingMatrix, &stiffnessMatrix);
	EXPECT_TRUE(forceVector.isZero());
	EXPECT_TRUE(massMatrix.isApprox(m_expectedMassMatrix));
	EXPECT_TRUE(dampingMatrix.isApprox(m_expectedDampingMatrix));
	EXPECT_TRUE(stiffnessMatrix.isApprox(m_expectedStiffnessMatrix));

	// Test addMatVec API with Mass component only
	forceVector.setZero();
	cube->addMatVec(m_restState, 1.0, 0.0, 0.0, m_vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 3 * 8; rowId++)
	{
		EXPECT_NEAR(m_expectedMassMatrix.row(rowId).sum(), forceVector[rowId], epsilon);
	}
	// Test addMatVec API with Damping component only
	forceVector.setZero();
	cube->addMatVec(m_restState, 0.0, 1.0, 0.0, m_vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 3 * 8; rowId++)
	{
		EXPECT_NEAR(m_expectedDampingMatrix.row(rowId).sum(), forceVector[rowId], epsilon);
	}
	// Test addMatVec API with Stiffness component only
	forceVector.setZero();
	cube->addMatVec(m_restState, 0.0, 0.0, 1.0, m_vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 3 * 8; rowId++)
	{
		EXPECT_NEAR(m_expectedStiffnessMatrix.row(rowId).sum(), forceVector[rowId], epsilon);
	}
	// Test addMatVec API with mix Mass/Damping/Stiffness components
	forceVector.setZero();
	cube->addMatVec(m_restState, 1.0, 2.0, 3.0, m_vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 3 * 8; rowId++)
	{
		double expectedCoef = 1.0 * m_expectedMassMatrix.row(rowId).sum() +
			2.0 * m_expectedDampingMatrix.row(rowId).sum() +
			3.0 * m_expectedStiffnessMatrix.row(rowId).sum();
		EXPECT_NEAR(expectedCoef, forceVector[rowId], epsilon);
	}
}
