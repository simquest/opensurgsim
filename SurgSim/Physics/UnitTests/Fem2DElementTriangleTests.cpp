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
#include "SurgSim/Math/GaussLegendreQuadrature.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem2DElementTriangle.h"

using SurgSim::Math::Matrix;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::Fem2DElementTriangle;

namespace
{
const double epsilon = 1e-8;
};

class MockFem2DElement : public Fem2DElementTriangle
{
public:
	MockFem2DElement(std::array<size_t, 3> nodeIds)
		: Fem2DElementTriangle(nodeIds)
	{
	}

	const Matrix33d& getInitialRotation() const
	{
		return m_initialRotation;
	}

	const Eigen::Matrix<double, 18, 18>& getInitialRotationTimes6()
	{
		const Matrix33d& R0 = getInitialRotation();

		m_initialRotationTimes6.setZero();

		SurgSim::Math::setSubMatrix(R0, 0, 0, 3, 3, &m_initialRotationTimes6);
		SurgSim::Math::setSubMatrix(R0, 1, 1, 3, 3, &m_initialRotationTimes6);
		SurgSim::Math::setSubMatrix(R0, 2, 2, 3, 3, &m_initialRotationTimes6);
		SurgSim::Math::setSubMatrix(R0, 3, 3, 3, 3, &m_initialRotationTimes6);
		SurgSim::Math::setSubMatrix(R0, 4, 4, 3, 3, &m_initialRotationTimes6);
		SurgSim::Math::setSubMatrix(R0, 5, 5, 3, 3, &m_initialRotationTimes6);

		return m_initialRotationTimes6;
	}

	const Eigen::Matrix<double, 3, 9> getBatozStrainDisplacement(double xi, double neta) const
	{
		return batozStrainDisplacement(xi, neta);
	}

	const Eigen::Matrix<double, 18, 18>& getLocalStiffnessMatrix() const
	{
		return m_KLocal;
	}

	const Eigen::Matrix<double, 18, 18>& getGlobalStiffnessMatrix() const
	{
		return m_K;
	}

	const Eigen::Matrix<double, 18, 18>& getLocalMassMatrix() const
	{
		return m_MLocal;
	}

	const Eigen::Matrix<double, 18, 18>& getGlobalMassMatrix() const
	{
		return m_M;
	}

	double getRestArea() const
	{
		return m_restArea;
	}

	const Eigen::Matrix<double, 18, 1>& getInitialPosition() const
	{
		return m_x0;
	}

	/// Get the Membrane (in-plane) shape functions parameters
	/// \param i The ith shape function
	/// \param ai, bi, ci The ith shape function parameters
	/// \note fi(x,y) = ai + x.bi + y.ci
	void getMembraneShapeFunction(int i, double *ai, double *bi, double *ci) const
	{
		*ai = m_membraneShapeFunctionsParameters(i, 0);
		*bi = m_membraneShapeFunctionsParameters(i, 1);
		*ci = m_membraneShapeFunctionsParameters(i, 2);
	}

	// The Thin-Plate shape functions (Batoz shape functions)
	// N1(xi, neta) = 2(1-xi-neta)(0.5-xi-neta)
	double batozN1(double xi, double neta) const { return 2.0 * (1.0 - xi - neta) * (0.5 - xi - neta); }
	// N2(xi, neta) = xi(2 xi-1)
	double batozN2(double xi, double neta) const { return xi * (2.0 * xi - 1.0);                       }
	// N3(xi, neta) = neta(2 neta-1)
	double batozN3(double xi, double neta) const { return neta * (2.0 * neta - 1.0);                   }
	// N4(xi, neta) = 4 xi neta
	double batozN4(double xi, double neta) const { return 4.0 * xi * neta;                             }
	// N5(xi, neta) = 4 neta(1-xi-neta)
	double batozN5(double xi, double neta) const { return 4.0 * neta * (1.0 - xi - neta);              }
	// N6(xi, neta) = 4 xi(1-xi-neta)
	double batozN6(double xi, double neta) const { return 4.0 * xi * (1.0 - xi - neta);                }

	// dN1/dxi(xi, neta) = 2[(-0.5+xi+neta) + (-1+xi+neta)] = 2(-3/2+2(xi+neta)) = 4(xi+neta) - 3
	double batozDN1Dxi(double xi, double neta) const { return 4.0 * (xi + neta) - 3.0;       }
	// dN2/dxi(xi, neta) = (2xi-1) + 2xi = 4xi-1
	double batozDN2Dxi(double xi, double neta) const { return 4.0 * xi - 1.0;                }
	// dN3/dxi(xi, neta) = 0
	double batozDN3Dxi(double xi, double neta) const { return 0.0;                           }
	// dN4/dxi(xi, neta) = 4 neta
	double batozDN4Dxi(double xi, double neta) const { return 4.0 * neta;                    }
	// dN5/dxi(xi,neta) = -4 neta
	double batozDN5Dxi(double xi, double neta) const { return -4.0 * neta;                   }
	// dN6/dxi(xi,neta) = 4(1-xi-neta) -4xi = 4(1-2xi-neta)
	double batozDN6Dxi(double xi, double neta) const { return 4.0 * (1.0 - 2.0 * xi - neta); }

	// dN1/dneta(xi, neta) = 2[(-0.5+xi+neta) + (-1+xi+neta)] = 2(-3/2 + 2xi + 2neta) = 4(xi+neta) - 3
	double batozDN1Dneta(double xi, double neta) const { return 4.0 * (xi + neta) - 3.0;       }
	// dN2/dneta(xi, neta) = 0
	double batozDN2Dneta(double xi, double neta) const { return 0.0;                           }
	// dN3/dneta(xi, neta) = 2neta-1 + 2neta = 4neta-1
	double batozDN3Dneta(double xi, double neta) const { return 4.0 * neta - 1.0;              }
	// dN4/dneta(xi, neta) = 4xi
	double batozDN4Dneta(double xi, double neta) const { return 4.0 * xi;                      }
	// dN5/dneta(xi, neta) = 4[(1-xi-neta) - neta] = 4(1-xi-2neta)
	double batozDN5Dneta(double xi, double neta) const { return 4.0 * (1.0 - xi - 2.0 * neta); }
	// dN6/dneta(xi, neta) = -4xi
	double batozDN6Dneta(double xi, double neta) const { return -4.0 * xi;                     }

	std::array<double, 9> batozHx(double xi, double neta)
	{
		std::array<double, 9> res;

		// 1.5 (a6N6-a5N5)
		res[0] = 1.5 * (m_ak[2] * batozN6(xi, neta) - m_ak[1] * batozN5(xi, neta));
		// b5N5+b6N6
		res[1] = m_bk[1] * batozN5(xi, neta) + m_bk[2] * batozN6(xi, neta);
		// N1 - c5N5 - c6N6
		res[2] = batozN1(xi, neta) - m_ck[1] * batozN5(xi, neta) - m_ck[2] * batozN6(xi, neta);

		// 1.5 (a4N4-a6N6)
		res[3] = 1.5 * (m_ak[0] * batozN4(xi, neta) - m_ak[2] * batozN6(xi, neta));
		// b6N6+b4N4
		res[4] = m_bk[2] * batozN6(xi, neta) + m_bk[0] * batozN4(xi, neta);
		// N2 - c6N6 - c4N4
		res[5] = batozN2(xi, neta) - m_ck[2] * batozN6(xi, neta) - m_ck[0] * batozN4(xi, neta);

		// 1.5 (a5N5-a4N4)
		res[6] = 1.5 * (m_ak[1] * batozN5(xi, neta) - m_ak[0] * batozN4(xi, neta));
		// b4N4+b5N5
		res[7] = m_bk[0] * batozN4(xi, neta) + m_bk[1] * batozN5(xi, neta);
		// N3 - c4N4 - c5N5
		res[8] = batozN3(xi, neta) - m_ck[0] * batozN4(xi, neta) - m_ck[1] * batozN5(xi, neta);

		return res;
	}
	std::array<double, 9> batozDhxDxiAlternative(double xi, double neta)
	{
		std::array<double, 9> res;

		// 1.5 (a6dN6-a5dN5)
		res[0] = 1.5 * (m_ak[2] * batozDN6Dxi(xi, neta) - m_ak[1] * batozDN5Dxi(xi, neta));
		// b5dN5+b6dN6
		res[1] = m_bk[1] * batozDN5Dxi(xi, neta) + m_bk[2] * batozDN6Dxi(xi, neta);
		// dN1 - c5dN5 - c6dN6
		res[2] = batozDN1Dxi(xi, neta) - m_ck[1] * batozDN5Dxi(xi, neta) - m_ck[2] * batozDN6Dxi(xi, neta);

		// 1.5 (a4dN4-a6dN6)
		res[3] = 1.5 * (m_ak[0] * batozDN4Dxi(xi, neta) - m_ak[2] * batozDN6Dxi(xi, neta));
		// b6dN6+b4dN4
		res[4] = m_bk[2] * batozDN6Dxi(xi, neta) + m_bk[0] * batozDN4Dxi(xi, neta);
		// dN2 - c6dN6 - c4dN4
		res[5] = batozDN2Dxi(xi, neta) - m_ck[2] * batozDN6Dxi(xi, neta) - m_ck[0] * batozDN4Dxi(xi, neta);

		// 1.5 (a5dN5-a4dN4)
		res[6] = 1.5 * (m_ak[1] * batozDN5Dxi(xi, neta) - m_ak[0] * batozDN4Dxi(xi, neta));
		// b4dN4+b5dN5
		res[7] = m_bk[0] * batozDN4Dxi(xi, neta) + m_bk[1] * batozDN5Dxi(xi, neta);
		// dN3 - c4dN4 - c5dN5
		res[8] = batozDN3Dxi(xi, neta) - m_ck[0] * batozDN4Dxi(xi, neta) - m_ck[1] * batozDN5Dxi(xi, neta);

		return res;
	}
	std::array<double, 9> batozDhxDnetaAlternative(double xi, double neta)
	{
		std::array<double, 9> res;

		// 1.5 (a6dN6-a5dN5)
		res[0] = 1.5 * (m_ak[2] * batozDN6Dneta(xi, neta) - m_ak[1] * batozDN5Dneta(xi, neta));
		// b5dN5+b6dN6
		res[1] = m_bk[1] * batozDN5Dneta(xi, neta) + m_bk[2] * batozDN6Dneta(xi, neta);
		// dN1 - c5dN5 - c6dN6
		res[2] = batozDN1Dneta(xi, neta) - m_ck[1] * batozDN5Dneta(xi, neta) - m_ck[2] * batozDN6Dneta(xi, neta);

		// 1.5 (a4dN4-a6dN6)
		res[3] = 1.5 * (m_ak[0] * batozDN4Dneta(xi, neta) - m_ak[2] * batozDN6Dneta(xi, neta));
		// b6dN6+b4dN4
		res[4] = m_bk[2] * batozDN6Dneta(xi, neta) + m_bk[0] * batozDN4Dneta(xi, neta);
		// dN2 - c6dN6 - c4dN4
		res[5] = batozDN2Dneta(xi, neta) - m_ck[2] * batozDN6Dneta(xi, neta) - m_ck[0] * batozDN4Dneta(xi, neta);

		// 1.5 (a5dN5-a4dN4)
		res[6] = 1.5 * (m_ak[1] * batozDN5Dneta(xi, neta) - m_ak[0] * batozDN4Dneta(xi, neta));
		// b4dN4+b5dN5
		res[7] = m_bk[0] * batozDN4Dneta(xi, neta) + m_bk[1] * batozDN5Dneta(xi, neta);
		// dN3 - c4dN4 - c5dN5
		res[8] = batozDN3Dneta(xi, neta) - m_ck[0] * batozDN4Dneta(xi, neta) - m_ck[1] * batozDN5Dneta(xi, neta);

		return res;
	}

	std::array<double, 9> batozHy(double xi, double neta)
	{
		std::array<double, 9> res;

		// 1.5 (d6N6-d5N5)
		res[0] = 1.5 * (m_dk[2] * batozN6(xi, neta) - m_dk[1] * batozN5(xi, neta));
		// -N1 + e5N5 + e6N6
		res[1] = -batozN1(xi, neta) + m_ek[1] * batozN5(xi, neta) + m_ek[2] * batozN6(xi, neta);
		// -b5N5-b6N6
		res[2] = -m_bk[1] * batozN5(xi, neta) - m_bk[2] * batozN6(xi, neta);

		// 1.5 (d4N4-d6N6)
		res[3] = 1.5 * (m_dk[0] * batozN4(xi, neta) - m_dk[2] * batozN6(xi, neta));
		// -N2 + e6N6 + e4N4
		res[4] = -batozN2(xi, neta) + m_ek[2] * batozN6(xi, neta) + m_ek[0] * batozN4(xi, neta);
		// -b6N6-b4N4
		res[5] = -m_bk[2] * batozN6(xi, neta) - m_bk[0] * batozN4(xi, neta);

		// 1.5 (d5N5-d4N4)
		res[6] = 1.5 * (m_dk[1] * batozN5(xi, neta) - m_dk[0] * batozN4(xi, neta));
		// -N3 + e4N4 + e5N5
		res[7] = -batozN3(xi, neta) + m_ek[0] * batozN4(xi, neta) + m_ek[1] * batozN5(xi, neta);
		// -b4N4-b5N5
		res[8] = -m_bk[0] * batozN4(xi, neta) - m_bk[1] * batozN5(xi, neta);

		return res;
	}
	std::array<double, 9> batozDhyDxiAlternative(double xi, double neta)
	{
		std::array<double, 9> res;

		// 1.5 (d6dN6-d5dN5)
		res[0] = 1.5 * (m_dk[2] * batozDN6Dxi(xi, neta) - m_dk[1] * batozDN5Dxi(xi, neta));
		// -dN1 + e5dN5 + e6dN6
		res[1] = -batozDN1Dxi(xi, neta) + m_ek[1] * batozDN5Dxi(xi, neta) + m_ek[2] * batozDN6Dxi(xi, neta);
		// -b5dN5-b6dN6
		res[2] = -m_bk[1] * batozDN5Dxi(xi, neta) - m_bk[2] * batozDN6Dxi(xi, neta);

		// 1.5 (d4dN4-d6dN6)
		res[3] = 1.5 * (m_dk[0] * batozDN4Dxi(xi, neta) - m_dk[2] * batozDN6Dxi(xi, neta));
		// -dN2 + e6dN6 + e4dN4
		res[4] = -batozDN2Dxi(xi, neta) + m_ek[2] * batozDN6Dxi(xi, neta) + m_ek[0] * batozDN4Dxi(xi, neta);
		// -b6dN6-b4dN4
		res[5] = -m_bk[2] * batozDN6Dxi(xi, neta) - m_bk[0] * batozDN4Dxi(xi, neta);

		// 1.5 (d5dN5-d4dN4)
		res[6] = 1.5 * (m_dk[1] * batozDN5Dxi(xi, neta) - m_dk[0] * batozDN4Dxi(xi, neta));
		// -dN3 + e4dN4 + e5dN5
		res[7] = -batozDN3Dxi(xi, neta) + m_ek[0] * batozDN4Dxi(xi, neta) + m_ek[1] * batozDN5Dxi(xi, neta);
		// -b4dN4-b5dN5
		res[8] = -m_bk[0] * batozDN4Dxi(xi, neta) - m_bk[1] * batozDN5Dxi(xi, neta);

		return res;
	}
	std::array<double, 9> batozDhyDnetaAlternative(double xi, double neta)
	{
		std::array<double, 9> res;

		// 1.5 (d6dN6-d5dN5)
		res[0] = 1.5 * (m_dk[2] * batozDN6Dneta(xi, neta) - m_dk[1] * batozDN5Dneta(xi, neta));
		// -dN1 + e5dN5 + e6dN6
		res[1] = -batozDN1Dneta(xi, neta) + m_ek[1] * batozDN5Dneta(xi, neta) + m_ek[2] * batozDN6Dneta(xi, neta);
		// -b5dN5-b6dN6
		res[2] = -m_bk[1] * batozDN5Dneta(xi, neta) - m_bk[2] * batozDN6Dneta(xi, neta);

		// 1.5 (d4dN4-d6dN6)
		res[3] = 1.5 * (m_dk[0] * batozDN4Dneta(xi, neta) - m_dk[2] * batozDN6Dneta(xi, neta));
		// -dN2 + e6dN6 + e4dN4
		res[4] = -batozDN2Dneta(xi, neta) + m_ek[2] * batozDN6Dneta(xi, neta) + m_ek[0] * batozDN4Dneta(xi, neta);
		// -b6dN6-b4dN4
		res[5] = -m_bk[2] * batozDN6Dneta(xi, neta) - m_bk[0] * batozDN4Dneta(xi, neta);

		// -dN3 + e4dN4 + e5dN5
		res[6] = 1.5 * (m_dk[1] * batozDN5Dneta(xi, neta) - m_dk[0] * batozDN4Dneta(xi, neta));
		// -dN3 + e4dN4 + e5dN5
		res[7] = -batozDN3Dneta(xi, neta) + m_ek[0] * batozDN4Dneta(xi, neta) + m_ek[1] * batozDN5Dneta(xi, neta);
		// -b4dN4-b5dN5
		res[8] = -m_bk[0] * batozDN4Dneta(xi, neta) - m_bk[1] * batozDN5Dneta(xi, neta);

		return res;
	}

	Eigen::Matrix<double, 3, 9> batozStrainDisplacementAlternativeDerivative(double xi, double neta)
	{
		Eigen::Matrix<double, 3, 9> res;
		std::array<double, 9> dHx_dxi, dHx_dneta, dHy_dxi, dHy_dneta;
		double coefficient = 1.0 / (2.0 * m_restArea);

		dHx_dxi   = batozDhxDxiAlternative(xi, neta);
		dHx_dneta = batozDhxDnetaAlternative(xi, neta);
		dHy_dxi   = batozDhyDxiAlternative(xi, neta);
		dHy_dneta = batozDhyDnetaAlternative(xi, neta);

		for(size_t i = 0; i < 9; ++i)
		{
			//  4 -> mid-edge 12
			//  5 -> mid-edge 20
			//  6 -> mid-edge 01
			res(0, i) = coefficient * ( m_yij[1] * dHx_dxi[i] + m_yij[2] * dHx_dneta[i]);
			res(1, i) = coefficient * (-m_xij[1] * dHy_dxi[i] - m_xij[2] * dHy_dneta[i]);
			res(2, i) = coefficient *
				(-m_xij[1] * dHx_dxi[i] - m_xij[2] * dHx_dneta[i] + m_yij[1] * dHy_dxi[i] + m_yij[2] * dHy_dneta[i]);
		}

		return res;
	}

	std::array<double, 9> batozFx(double xi, double neta,
		double (MockFem2DElement::*f1)(double,double) const,
		double (MockFem2DElement::*f2)(double,double) const,
		double (MockFem2DElement::*f3)(double,double) const,
		double (MockFem2DElement::*f4)(double,double) const,
		double (MockFem2DElement::*f5)(double,double) const,
		double (MockFem2DElement::*f6)(double,double) const)
	{
		std::array<double, 9> res;

		// 1.5 (a6N6-a5N5)
		res[0] = 1.5 * (m_ak[2] * (this->*f6)(xi, neta) - m_ak[1] * (this->*f5)(xi, neta));
		// b5N5+b6N6
		res[1] = m_bk[1] * (this->*f5)(xi, neta) + m_bk[2] * (this->*f6)(xi, neta);
		// N1 - c5N5 - c6N6
		res[2] = (this->*f1)(xi, neta) - m_ck[1] * (this->*f5)(xi, neta) - m_ck[2] * (this->*f6)(xi, neta);

		// 1.5 (a4N4-a6N6)
		res[3] = 1.5 * (m_ak[0] * (this->*f4)(xi, neta) - m_ak[2] * (this->*f6)(xi, neta));
		// b6N6+b4N4
		res[4] = m_bk[2] * (this->*f6)(xi, neta) + m_bk[0] * (this->*f4)(xi, neta);
		// N2 - c6N6 - c4N4
		res[5] = (this->*f2)(xi, neta) - m_ck[2] * (this->*f6)(xi, neta) - m_ck[0] * (this->*f4)(xi, neta);

		// 1.5 (a5N5-a4N4)
		res[6] = 1.5 * (m_ak[1] * (this->*f5)(xi, neta) - m_ak[0] * (this->*f4)(xi, neta));
		// b4N4+b5N5
		res[7] = m_bk[0] * (this->*f4)(xi, neta) + m_bk[1] * (this->*f5)(xi, neta);
		// N3 - c4N4 - c5N5
		res[8] = (this->*f3)(xi, neta) - m_ck[0] * (this->*f4)(xi, neta) - m_ck[1] * (this->*f5)(xi, neta);

		return res;
	}

	std::array<double, 9> batozFy(double xi, double neta,
		double (MockFem2DElement::*f1)(double,double) const,
		double (MockFem2DElement::*f2)(double,double) const,
		double (MockFem2DElement::*f3)(double,double) const,
		double (MockFem2DElement::*f4)(double,double) const,
		double (MockFem2DElement::*f5)(double,double) const,
		double (MockFem2DElement::*f6)(double,double) const)
	{
		std::array<double, 9> res;

		// 1.5 (d6N6-d5N5)
		res[0] = 1.5 * (m_dk[2] * (this->*f6)(xi, neta) - m_dk[1] * (this->*f5)(xi, neta));
		// -N1 + e5N5 + e6N6
		res[1] = -(this->*f1)(xi, neta) + m_ek[1] * (this->*f5)(xi, neta) + m_ek[2] * (this->*f6)(xi, neta);
		// -b5N5-b6N6
		res[2] = -m_bk[1] * (this->*f5)(xi, neta) - m_bk[2] * (this->*f6)(xi, neta);

		// 1.5 (d4N4-d6N6)
		res[3] = 1.5 * (m_dk[0] * (this->*f4)(xi, neta) - m_dk[2] * (this->*f6)(xi, neta));
		// -N2 + e6N6 + e4N4
		res[4] = -(this->*f2)(xi, neta) + m_ek[2] * (this->*f6)(xi, neta) + m_ek[0] * (this->*f4)(xi, neta);
		// -b6N6-b4N4
		res[5] = -m_bk[2] * (this->*f6)(xi, neta) - m_bk[0] * (this->*f4)(xi, neta);

		// 1.5 (d5N5-d4N4)
		res[6] = 1.5 * (m_dk[1] * (this->*f5)(xi, neta) - m_dk[0] * (this->*f4)(xi, neta));
		// -N3 + e4N4 + e5N5
		res[7] = -(this->*f3)(xi, neta) + m_ek[0] * (this->*f4)(xi, neta) + m_ek[1] * (this->*f5)(xi, neta);
		// -b4N4-b5N5
		res[8] = -m_bk[0] * (this->*f4)(xi, neta) - m_bk[1] * (this->*f5)(xi, neta);

		return res;
	}

	Eigen::Matrix<double, 3, 9> batozStrainDisplacementNumericalDerivation(double xi, double neta)
	{
		Eigen::Matrix<double, 3, 9> res;
		std::array<double, 9> dHx_dxi, dHx_dneta, dHy_dxi, dHy_dneta;
		double coefficient = 1.0 / (2.0 * m_restArea);

		dHx_dxi = batozFx(xi, neta, &MockFem2DElement::batozDN1Dxi , &MockFem2DElement::batozDN2Dxi ,
			&MockFem2DElement::batozDN3Dxi , &MockFem2DElement::batozDN4Dxi , &MockFem2DElement::batozDN5Dxi ,
			&MockFem2DElement::batozDN6Dxi);
		dHx_dneta = batozFx(xi, neta, &MockFem2DElement::batozDN1Dneta, &MockFem2DElement::batozDN2Dneta,
			&MockFem2DElement::batozDN3Dneta, &MockFem2DElement::batozDN4Dneta, &MockFem2DElement::batozDN5Dneta,
			&MockFem2DElement::batozDN6Dneta);
		dHy_dxi = batozFy(xi, neta, &MockFem2DElement::batozDN1Dxi, &MockFem2DElement::batozDN2Dxi ,
			&MockFem2DElement::batozDN3Dxi, &MockFem2DElement::batozDN4Dxi, &MockFem2DElement::batozDN5Dxi,
			&MockFem2DElement::batozDN6Dxi);
		dHy_dneta = batozFy(xi, neta, &MockFem2DElement::batozDN1Dneta, &MockFem2DElement::batozDN2Dneta,
			&MockFem2DElement::batozDN3Dneta, &MockFem2DElement::batozDN4Dneta, &MockFem2DElement::batozDN5Dneta,
			&MockFem2DElement::batozDN6Dneta);

		for(size_t i = 0; i < 9; ++i)
		{
			//  4 -> mid-edge 12
			//  5 -> mid-edge 20
			//  6 -> mid-edge 01
			res(0, i) = coefficient * ( m_yij[1] * dHx_dxi[i] + m_yij[2] * dHx_dneta[i]);
			res(1, i) = coefficient * (-m_xij[1] * dHy_dxi[i] - m_xij[2] * dHy_dneta[i]);
			res(2, i) = coefficient *
				(-m_xij[1] * dHx_dxi[i] - m_xij[2] * dHx_dneta[i] + m_yij[1] * dHy_dxi[i] + m_yij[2] * dHy_dneta[i]);
		}

		return res;
	}

private:
	Eigen::Matrix<double, 18, 18> m_initialRotationTimes6;
};

class Fem2DElementTriangleTests : public ::testing::Test
{
public:
	static const int m_numberNodes = 6;

	std::array<size_t, 3> m_nodeIds;
	SurgSim::Math::OdeState m_restState;
	double m_expectedVolume;
	double m_rho, m_E, m_nu;
	double m_A;         // area
	double m_thickness; // thickness
	Quaterniond m_rotation, m_expectedRotation;
	Eigen::Matrix<double, 18, 1> m_expectedX0;

	virtual void SetUp() override
	{
		using SurgSim::Math::getSubVector;

		m_rho = 1000.0;
		m_E = 1e9;
		m_nu = 0.45;
		m_thickness = 1e-2;
		m_A = 1.0 / 2.0;
		m_expectedVolume = m_A * m_thickness;

		// Triangle is made of node 3, 1 and 5 in a bigger system containing m_numberNodes nodes (at least 6)
		m_nodeIds[0] = 3;
		m_nodeIds[1] = 1;
		m_nodeIds[2] = 5;

		m_restState.setNumDof(6, m_numberNodes);

		m_rotation.coeffs().setRandom();
		m_rotation.normalize();

		Vector3d A(0.0, 0.0, 0.0);
		Vector3d B(1.0, 0.0, 0.0);
		Vector3d C(0.0, 1.0, 0.0);
		Vector& x = m_restState.getPositions();
		getSubVector(x, m_nodeIds[0], 6).segment<3>(0) = m_rotation._transformVector(A);
		getSubVector(x, m_nodeIds[1], 6).segment<3>(0) = m_rotation._transformVector(B);
		getSubVector(x, m_nodeIds[2], 6).segment<3>(0) = m_rotation._transformVector(C);

		for (size_t nodeId = 0; nodeId < 3; ++nodeId)
		{
			m_expectedX0.segment(6 * nodeId, 6) = getSubVector(x, m_nodeIds[nodeId], 6);
		}

		// The initial rotation of ABC is defined by (i=AB(1 0 0), j=AC(0 1 0), k=AB^AC(0 0 1)) = Identity
		// Therefore, by applying m_rotation to the triangle, the initial rotation of the element should be m_rotation
		m_expectedRotation = m_rotation;
	}

	void getExpectedLocalMassMatrix(Eigen::Ref<SurgSim::Math::Matrix> mass)
	{
		double m = m_rho * m_thickness * m_A;

		mass.setIdentity();

		mass.block<2, 2>(0, 0).diagonal().setConstant(m / 6.0);
		mass.block<2, 2>(0, 6).diagonal().setConstant(m / 12.0);
		mass.block<2, 2>(0, 12).diagonal().setConstant(m / 12.0);
		mass.block<2, 2>(3, 3).setConstant(-6.25e-6);
		mass.block<2, 2>(3, 3).diagonal().setConstant(6.0416666666666666e-5);

		mass.block<2, 2>(6, 0).diagonal().setConstant(m / 12.0);
		mass.block<2, 2>(6, 6).diagonal().setConstant(m / 6.0);
		mass.block<2, 2>(6, 12).diagonal().setConstant(m / 12.0);
		mass.block<2, 2>(9, 9).setConstant(-6.25e-6);
		mass.block<2, 2>(9, 9).diagonal().setConstant(6.0416666666666666e-5);

		mass.block<2, 2>(12, 0).diagonal().setConstant(m / 12.0);
		mass.block<2, 2>(12, 6).diagonal().setConstant(m / 12.0);
		mass.block<2, 2>(12, 12).diagonal().setConstant(m / 6.0);
		mass.block<2, 2>(15, 15).setConstant(-6.25e-6);
		mass.block<2, 2>(15, 15).diagonal().setConstant(6.0416666666666666e-5);
	}

	void getExpectedLocalStiffnessMatrix(Eigen::Ref<SurgSim::Math::Matrix> stiffness)
	{
		typedef Eigen::Matrix<double, 9, 9> Matrix99Type;
		typedef Eigen::Matrix<double, 6, 6> Matrix66Type;

		Matrix66Type membraneStiffness = getMembraneLocalStiffnessMatrix();
		Matrix99Type plateStiffness = getPlateLocalStiffnessMatrix();

		// Assemble the membrane and plane stiffness
		stiffness.setIdentity();
		for(size_t row = 0; row < 3; ++row)
		{
			for(size_t column = 0; column < 3; ++column)
			{
				// Membrane part
				stiffness.block(6 * row, 6 * column, 2, 2) = membraneStiffness.block(2 * row, 2 * column, 2, 2);

				// Thin-plate part
				stiffness.block(6 * row + 2, 6 * column + 2, 3, 3) = plateStiffness.block(3 * row, 3 * column, 3, 3);
			}
		}
	}

	Eigen::Matrix<double, 6, 6> getMembraneLocalStiffnessMatrix()
	{
		typedef Eigen::Matrix<double, 3, 6> Matrix36Type;

		std::shared_ptr<MockFem2DElement> element = getElement();

		// Membrane theory (using "Theory of Matrix Structural Analysis" - Przemieniecki)
		// ux = c1.x + c2.y + c3
		// uy = c4.x + c5.y + c6
		// ux(x1, y1) = u1x = c1.x1 + c2.y1 + c3     (u1x) (x1 y1 1)(c1)
		// ux(x2, y2) = u2x = c1.x2 + c2.y2 + c3 <=> (u2x)=(x2 y2 1)(c2)
		// ux(x3, y3) = u3x = c1.x3 + c2.y3 + c3     (u3x) (x3 y3 1)(c3)
		// <=> (c1) = 1/det( y23       -y13          y12      )(u1x)
		//     (c2)        (-x23        x13         -x12      )(u2x)
		//     (c3)        ( x2y3-x3y2 -(x1y3-x3y1)  x1y2-x2y1)(u3x)
		// det = (x1y2 + x2y3 + x3y1 - x3y2 - x2y1 - x1y3)
		// = x21(y3) - y21(x3) +x2(-y1) - y2(-x1) - x1(-y1) - (-y1)(-x1) = x21y31 - y21x31 = 2A > 0
		//
		// and similarily for uy
		// <=> (c4) = 1/(2A)( y23       -y13          y12      )(u1y)
		//     (c5)         (-x23        x13         -x12      )(u2y)
		//     (c6)         ( x2y3-x3y2 -(x1y3-x3y1)  x1y2-x2y1)(u3y)
		//
		// Therefore ux = 1/(2A) [x.(y23.u1x - y13.u2x + y12.u3x) + y.(-x23.u1x + x13.u2x - x12.u3x) + constant]
		// Exx = dux/dx = 1/(2A) (y23.u1x - y13.u2x + y12.u3x) = b.u
		// Therefore uy = 1/(2A) [x.(y23.u1y - y13.u2y + y12.u3y) + y.(-x23.u1y + x13.u2y - x12.u3y) + constant]
		// Eyy = duy/dy = 1/(2A) (-x23.u1y + x13.u2y - x12.u3y) = b.u
		// Exy = dux/dy + duy/dx = 1/(2A) (-x23.u1x + x13.u2x - x12.u3x + y23.u1y - y13.u2y + y12.u3y) = b.u
		Vector3d A2D = m_expectedRotation.inverse()._transformVector(m_expectedX0.segment(0,3));
		Vector3d B2D = m_expectedRotation.inverse()._transformVector(m_expectedX0.segment(6,3));
		Vector3d C2D = m_expectedRotation.inverse()._transformVector(m_expectedX0.segment(12,3));
		double x12 = A2D[0] - B2D[0];
		double x13 = A2D[0] - C2D[0];
		double x23 = B2D[0] - C2D[0];
		double y12 = A2D[1] - B2D[1];
		double y13 = A2D[1] - C2D[1];
		double y23 = B2D[1] - C2D[1];
		Matrix36Type b = Matrix36Type::Zero();
		b(0, 0) = y23;
		b(0, 2) = -y13;
		b(0, 4) = y12;
		b(1, 1) = -x23;
		b(1, 3) = x13;
		b(1, 5) = -x12;
		b(2, 0) = -x23;
		b(2, 1) = y23;
		b(2, 2) = x13;
		b(2, 3) = -y13;
		b(2, 4) = -x12;
		b(2, 5) = y12;
		b *= 1.0 / (2.0 * m_A);
		Matrix33d Emembrane;
		Emembrane << 1.0, m_nu, 0.0, m_nu, 1.0, 0.0, 0.0, 0.0, (1.0 - m_nu) / 2.0;
		Emembrane *= m_E / (1.0 - m_nu * m_nu);
		return (m_thickness * m_A) * b.transpose() * Emembrane * b;
	}

	Eigen::Matrix<double, 9, 9> getPlateLocalStiffnessMatrix()
	{
		typedef Eigen::Matrix<double, 3, 9> Matrix39Type;

		Eigen::Matrix<double, 9, 9> stiffness;
		std::shared_ptr<MockFem2DElement> element = getElement();

		// Thin-plate theory (Batoz)
		Matrix39Type B0 = element->batozStrainDisplacementNumericalDerivation(0.5, 0.0);
		Matrix39Type B1 = element->batozStrainDisplacementNumericalDerivation(0.0, 0.5);
		Matrix39Type B2 = element->batozStrainDisplacementNumericalDerivation(0.5, 0.5);
		Matrix33d Eplate;
		Eplate << 1.0, m_nu, 0.0, m_nu, 1.0, 0.0, 0.0, 0.0, (1.0 - m_nu) / 2.0;
		Eplate *= m_E * m_thickness * m_thickness * m_thickness / (12.0 * (1.0 - m_nu * m_nu));
		// Integration using 3 Gauss point on the mid-point of each triangle edge
		// weight = A/3 for all 3 (A is the area of the parametrized triangle = 0.5)
		stiffness = (1.0 / 6.0) * B0.transpose() * Eplate * B0;
		stiffness += (1.0 / 6.0) * B1.transpose() * Eplate * B1;
		stiffness += (1.0 / 6.0) * B2.transpose() * Eplate * B2;
		stiffness *= 2.0 * m_A;

		return stiffness;
	}

	std::shared_ptr<MockFem2DElement> getElement()
	{
		auto element = std::make_shared<MockFem2DElement>(m_nodeIds);
		element->setThickness(m_thickness);
		element->setMassDensity(m_rho);
		element->setPoissonRatio(m_nu);
		element->setYoungModulus(m_E);
		element->initialize(m_restState);
		return element;
	}
};

TEST_F(Fem2DElementTriangleTests, ConstructorTest)
{
	ASSERT_NO_THROW({ MockFem2DElement triangle(m_nodeIds); });
}

TEST_F(Fem2DElementTriangleTests, NodeIdsTest)
{
	Fem2DElementTriangle element(m_nodeIds);
	EXPECT_EQ(3u, element.getNumNodes());
	EXPECT_EQ(3u, element.getNodeIds().size());
	for (int i = 0; i < 3; i++)
	{
		EXPECT_EQ(m_nodeIds[i], element.getNodeId(i));
		EXPECT_EQ(m_nodeIds[i], element.getNodeIds()[i]);
	}
}

TEST_F(Fem2DElementTriangleTests, setGetThicknessTest)
{
	Fem2DElementTriangle element(m_nodeIds);

	// Default thickness = 0.0
	EXPECT_DOUBLE_EQ(0.0, element.getThickness());
	// Set to a valid thickness
	element.setThickness(1.54);
	EXPECT_DOUBLE_EQ(1.54, element.getThickness());
	// Set to an invalid thickness
	EXPECT_ANY_THROW(element.setThickness(0.0));
	EXPECT_ANY_THROW(element.setThickness(-9.4));
}

TEST_F(Fem2DElementTriangleTests, MaterialParameterTest)
{
	Fem2DElementTriangle element(m_nodeIds);
	element.setThickness(m_thickness);

	// Test the various mode of failure related to the physical parameters
	// This has been already tested in FemElementTests, but this is to make sure this method is called properly
	// So the same behavior should be expected
	{
		// Mass density not set
		ASSERT_ANY_THROW(element.initialize(m_restState));

		// Poisson Ratio not set
		element.setMassDensity(-1234.56);
		ASSERT_ANY_THROW(element.initialize(m_restState));

		// Young modulus not set
		element.setPoissonRatio(0.55);
		ASSERT_ANY_THROW(element.initialize(m_restState));

		// Invalid mass density
		element.setYoungModulus(-4321.33);
		ASSERT_ANY_THROW(element.initialize(m_restState));

		// Invalid Poisson ratio
		element.setMassDensity(m_rho);
		ASSERT_ANY_THROW(element.initialize(m_restState));

		// Invalid Young modulus
		element.setPoissonRatio(m_nu);
		ASSERT_ANY_THROW(element.initialize(m_restState));

		element.setYoungModulus(m_E);
		ASSERT_NO_THROW(element.initialize(m_restState));
	}
}

TEST_F(Fem2DElementTriangleTests, VolumeTest)
{
	std::shared_ptr<MockFem2DElement> element = getElement();
	EXPECT_NEAR(element->getVolume(m_restState), m_expectedVolume, 1e-10);
}

TEST_F(Fem2DElementTriangleTests, CoordinateTests)
{
	Fem2DElementTriangle element(m_nodeIds);

	Vector validNaturalCoordinate(3);
	Vector validNaturalCoordinate2(3);
	Vector invalidNaturalCoordinateSumNot1(3);
	Vector invalidNaturalCoordinateNegativeValue(3);
	Vector invalidNaturalCoordinateBiggerThan1Value(3);
	Vector invalidNaturalCoordinateSize2(2), invalidNaturalCoordinateSize4(4);

	validNaturalCoordinate << 0.4, 0.5, 0.1;
	validNaturalCoordinate2 << -1e-11, 1.0 + 1e-11, 0.0;
	invalidNaturalCoordinateSumNot1 << 0.4, 0.5, 0.3;
	invalidNaturalCoordinateNegativeValue << 0.7, 0.7, -0.4;
	invalidNaturalCoordinateBiggerThan1Value << 1.4, 0.6, -1.0;
	invalidNaturalCoordinateSize2 << 0.4, 0.6;
	invalidNaturalCoordinateSize4 << 0.2, 0.2, 0.2, 0.4;
	EXPECT_TRUE(element.isValidCoordinate(validNaturalCoordinate));
	EXPECT_TRUE(element.isValidCoordinate(validNaturalCoordinate2));
	EXPECT_FALSE(element.isValidCoordinate(invalidNaturalCoordinateSumNot1));
	EXPECT_FALSE(element.isValidCoordinate(invalidNaturalCoordinateNegativeValue));
	EXPECT_FALSE(element.isValidCoordinate(invalidNaturalCoordinateBiggerThan1Value));
	EXPECT_FALSE(element.isValidCoordinate(invalidNaturalCoordinateSize2));
	EXPECT_FALSE(element.isValidCoordinate(invalidNaturalCoordinateSize4));

	Vector naturalCoordinateA(3), naturalCoordinateB(3), naturalCoordinateC(3), naturalCoordinateMiddle(3);
	Vector ptA, ptB, ptC, ptMiddle;
	naturalCoordinateA << 1.0, 0.0, 0.0;
	naturalCoordinateB << 0.0, 1.0, 0.0;
	naturalCoordinateC << 0.0, 0.0, 1.0;
	naturalCoordinateMiddle << 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0;
	EXPECT_THROW(ptA = element.computeCartesianCoordinate(m_restState, invalidNaturalCoordinateBiggerThan1Value), \
		SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(ptA = element.computeCartesianCoordinate(m_restState, invalidNaturalCoordinateNegativeValue), \
		SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(ptA = element.computeCartesianCoordinate(m_restState, invalidNaturalCoordinateSize2), \
		SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(ptA = element.computeCartesianCoordinate(m_restState, invalidNaturalCoordinateSize4), \
		SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(ptA = element.computeCartesianCoordinate(m_restState, invalidNaturalCoordinateSumNot1), \
		SurgSim::Framework::AssertionFailure);
	EXPECT_NO_THROW(ptA = element.computeCartesianCoordinate(m_restState, naturalCoordinateA));
	EXPECT_NO_THROW(ptB = element.computeCartesianCoordinate(m_restState, naturalCoordinateB));
	EXPECT_NO_THROW(ptC = element.computeCartesianCoordinate(m_restState, naturalCoordinateC));
	EXPECT_NO_THROW(ptMiddle = element.computeCartesianCoordinate(m_restState, naturalCoordinateMiddle));
	EXPECT_TRUE(ptA.isApprox(m_rotation._transformVector(Vector3d(0.0, 0.0, 0.0))));
	EXPECT_TRUE(ptB.isApprox(m_rotation._transformVector(Vector3d(1.0, 0.0, 0.0))));
	EXPECT_TRUE(ptC.isApprox(m_rotation._transformVector(Vector3d(0.0, 1.0, 0.0))));
	EXPECT_TRUE(ptMiddle.isApprox(m_rotation._transformVector(Vector3d(1.0 / 3.0, 1.0 / 3.0, 0.0))));

	Vector3d cartesian = m_rotation._transformVector(Vector3d(0.0, 0.0, 0.0));
	EXPECT_THROW(element.computeNaturalCoordinate(m_restState, cartesian), SurgSim::Framework::AssertionFailure);
}

TEST_F(Fem2DElementTriangleTests, RestAreaTest)
{
	std::shared_ptr<MockFem2DElement> element = getElement();
	EXPECT_NEAR(element->getRestArea(), m_A, 1e-10);
}

TEST_F(Fem2DElementTriangleTests, InitialRotationTest)
{
	std::shared_ptr<MockFem2DElement> element = getElement();

	// Use a mask to test the structure of the rotation matrix R0 (6 digonal block 3x3 matrix and 0 elsewhere)
	Eigen::Matrix<double, 18, 18> mask;
	mask.setOnes();
	mask.block<3, 3>(0, 0).setZero();
	mask.block<3, 3>(3, 3).setZero();
	mask.block<3, 3>(6, 6).setZero();
	mask.block<3, 3>(9, 9).setZero();
	mask.block<3, 3>(12, 12).setZero();
	mask.block<3, 3>(15, 15).setZero();
	EXPECT_TRUE(element->getInitialRotationTimes6().cwiseProduct(mask).isZero());

	EXPECT_TRUE(element->getInitialRotationTimes6().block(0, 0, 3, 3).isApprox(m_expectedRotation.matrix()));
	EXPECT_TRUE(element->getInitialRotationTimes6().block(3, 3, 3, 3).isApprox(m_expectedRotation.matrix()));
	EXPECT_TRUE(element->getInitialRotationTimes6().block(6, 6, 3, 3).isApprox(m_expectedRotation.matrix()));
	EXPECT_TRUE(element->getInitialRotationTimes6().block(9, 9, 3, 3).isApprox(m_expectedRotation.matrix()));
	EXPECT_TRUE(element->getInitialRotationTimes6().block(12, 12, 3, 3).isApprox(m_expectedRotation.matrix()));
	EXPECT_TRUE(element->getInitialRotationTimes6().block(15, 15, 3, 3).isApprox(m_expectedRotation.matrix()));
}

TEST_F(Fem2DElementTriangleTests, StrainDisplacementPlateAtGaussPointTest)
{
	std::shared_ptr<MockFem2DElement> element = getElement();

	Eigen::Matrix<double, 3, 9> strainDisplacement[3];
	strainDisplacement[0] = element->getBatozStrainDisplacement(0.0, 0.5);
	strainDisplacement[1] = element->getBatozStrainDisplacement(0.5, 0.0);
	strainDisplacement[2] = element->getBatozStrainDisplacement(0.5, 0.5);

	Eigen::Matrix<double, 3, 9> strainDisplacementExpected1[3];
	strainDisplacementExpected1[0] = element->batozStrainDisplacementAlternativeDerivative(0.0, 0.5);
	strainDisplacementExpected1[1] = element->batozStrainDisplacementAlternativeDerivative(0.5, 0.0);
	strainDisplacementExpected1[2] = element->batozStrainDisplacementAlternativeDerivative(0.5, 0.5);
	Eigen::Matrix<double, 3, 9> strainDisplacementExpected2[3];
	strainDisplacementExpected2[0] = element->batozStrainDisplacementNumericalDerivation(0.0, 0.5);
	strainDisplacementExpected2[1] = element->batozStrainDisplacementNumericalDerivation(0.5, 0.0);
	strainDisplacementExpected2[2] = element->batozStrainDisplacementNumericalDerivation(0.5, 0.5);

	// Validate the alternative technique against the numerical evaluation
	EXPECT_TRUE(strainDisplacementExpected1[0].isApprox(strainDisplacementExpected2[0])) <<
		strainDisplacementExpected1[0] << std::endl <<
		strainDisplacementExpected2[0] << std::endl;
	EXPECT_TRUE(strainDisplacementExpected1[1].isApprox(strainDisplacementExpected2[1])) <<
		strainDisplacementExpected1[1] << std::endl <<
		strainDisplacementExpected2[1] << std::endl;
	EXPECT_TRUE(strainDisplacementExpected1[2].isApprox(strainDisplacementExpected2[2])) <<
		strainDisplacementExpected1[2] << std::endl <<
		strainDisplacementExpected2[2] << std::endl;

	// Validate the Fem2DElementTriangle internal calculation against both technique
	EXPECT_TRUE(strainDisplacement[0].isApprox(strainDisplacementExpected1[0])) <<
		strainDisplacement[0] << std::endl <<
		strainDisplacementExpected1[0] << std::endl;
	EXPECT_TRUE(strainDisplacement[0].isApprox(strainDisplacementExpected2[0])) <<
		strainDisplacement[0] << std::endl <<
		strainDisplacementExpected2[0] << std::endl;

	EXPECT_TRUE(strainDisplacement[1].isApprox(strainDisplacementExpected1[1])) <<
		strainDisplacement[1] << std::endl <<
		strainDisplacementExpected1[1] << std::endl;
	EXPECT_TRUE(strainDisplacement[1].isApprox(strainDisplacementExpected2[1])) <<
		strainDisplacement[1] << std::endl <<
		strainDisplacementExpected2[1] << std::endl;

	EXPECT_TRUE(strainDisplacement[2].isApprox(strainDisplacementExpected1[2])) <<
		strainDisplacement[2] << std::endl <<
		strainDisplacementExpected1[2] << std::endl;
	EXPECT_TRUE(strainDisplacement[2].isApprox(strainDisplacementExpected2[2])) <<
		strainDisplacement[2] << std::endl <<
		strainDisplacementExpected2[2] << std::endl;
}

namespace
{
/// Shape function evaluation Ni(x,y) = ai + bi.x + ci.y
/// \param i Defines which shape function to evaluate
/// \param ai, bi, ci The shape functions parameters
/// \param p The 2D point (x, y) to evaluate the shape function at
/// \return The shape function evaluation ai + bi.x + ci.y
double N(size_t i,
	const std::array<double, 3>& ai, const std::array<double, 3>& bi, const std::array<double, 3>& ci,
	const SurgSim::Math::Vector2d& p)
{
	return ai[i] + bi[i] * p[0] + ci[i] * p[1];
}
};

TEST_F(Fem2DElementTriangleTests, MembraneShapeFunctionsTest)
{
	using SurgSim::Math::getSubVector;

	std::shared_ptr<MockFem2DElement> tri = getElement();

	EXPECT_TRUE(tri->getInitialPosition().isApprox(m_expectedX0)) <<
		"x0 = " << tri->getInitialPosition().transpose() << std::endl << "x0 expected = " << m_expectedX0.transpose();

	// Ni(x,y) = (ai + bi.x + ci.y)
	std::array<double, 3> ai, bi, ci;
	for (int i = 0; i < 3; ++i)
	{
		tri->getMembraneShapeFunction(i, &(ai[i]), &(bi[i]), &(ci[i]));
	}

	// We should (in local frame) have by construction:
	// { N0(p0) = 1    N1(p0)=N2(p0)=N3(p0)=0
	// { N1(p1) = 1    N1(p1)=N2(p1)=N3(p1)=0
	// { N2(p2) = 1    N1(p2)=N2(p2)=N3(p2)=0
	// { N3(p3) = 1    N1(p3)=N2(p3)=N3(p3)=0
	const Vector3d p0 = getSubVector(m_expectedX0, 0, 6).segment(0, 3);
	const Vector3d p1 = getSubVector(m_expectedX0, 1, 6).segment(0, 3);
	const Vector3d p2 = getSubVector(m_expectedX0, 2, 6).segment(0, 3);
	SurgSim::Math::Vector2d p02D = m_expectedRotation.inverse()._transformVector(p0).segment(0, 2);
	SurgSim::Math::Vector2d p12D = m_expectedRotation.inverse()._transformVector(p1).segment(0, 2);
	SurgSim::Math::Vector2d p22D = m_expectedRotation.inverse()._transformVector(p2).segment(0, 2);
	std::array<double, 3> Ni_p0, Ni_p1, Ni_p2;
	for (int i = 0; i < 3; i++)
	{
		Ni_p0[i] = N(i, ai, bi, ci, p02D);
		Ni_p1[i] = N(i, ai, bi, ci, p12D);
		Ni_p2[i] = N(i, ai, bi, ci, p22D);
	}
	EXPECT_NEAR(Ni_p0[0], 1.0, 1e-12);
	EXPECT_NEAR(Ni_p0[1], 0.0, 1e-12);
	EXPECT_NEAR(Ni_p0[2], 0.0, 1e-12);

	EXPECT_NEAR(Ni_p1[0], 0.0, 1e-12);
	EXPECT_NEAR(Ni_p1[1], 1.0, 1e-12);
	EXPECT_NEAR(Ni_p1[2], 0.0, 1e-12);

	EXPECT_NEAR(Ni_p2[0], 0.0, 1e-12);
	EXPECT_NEAR(Ni_p2[1], 0.0, 1e-12);
	EXPECT_NEAR(Ni_p2[2], 1.0, 1e-12);

	// We should have the relation sum(Ni(x,y) = 1) for all points in the triangle
	// We verify that relation by sampling the tetrahedron volume
	for (double sp0p1 = 0; sp0p1 <= 1.0; sp0p1+=0.1)
	{
		for (double sp0p2 = 0; sp0p1 + sp0p2 <= 1.0; sp0p2+=0.1)
		{
			Vector3d p = p0 + sp0p1 * (p1 - p0) + sp0p2 * (p2 - p0);
			SurgSim::Math::Vector2d p2D = m_expectedRotation.inverse()._transformVector(p).segment(0, 2);
			std::array<double, 3> Ni_p;
			for (size_t i = 0; i < 3; ++i)
			{
				Ni_p[i] = N(i, ai, bi, ci, p2D);
			}
			EXPECT_NEAR(Ni_p[0] + Ni_p[1] + Ni_p[2], 1.0, 1e-10) <<
				" for sp0p1 = " << sp0p1 << ", sp0p2 = " << sp0p2 << std::endl <<
				" N0(x,y,z) = " << Ni_p[0] << " N1(x,y,z) = " << Ni_p[1] << " N2(x,y,z) = " << Ni_p[2];
		}
	}
}

TEST_F(Fem2DElementTriangleTests, PlateShapeFunctionsTest)
{
	std::shared_ptr<MockFem2DElement> tri = getElement();

	// Shape function N1 weigth point 0 (parametric coordinate 0 0)
	EXPECT_DOUBLE_EQ(1.0, tri->batozN1(0.0, 0.0));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN1(1.0, 0.0));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN1(0.0, 1.0));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN1(0.5, 0.5));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN1(0.0, 0.5));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN1(0.5, 0.0));

	// Shape function N2 weigth point 1 (parametric coordinate 1 0)
	EXPECT_DOUBLE_EQ(0.0, tri->batozN2(0.0, 0.0));
	EXPECT_DOUBLE_EQ(1.0, tri->batozN2(1.0, 0.0));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN2(0.0, 1.0));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN2(0.5, 0.5));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN2(0.0, 0.5));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN2(0.5, 0.0));

	// Shape function N3 weigth point 2 (parametric coordinate 0 1)
	EXPECT_DOUBLE_EQ(0.0, tri->batozN3(0.0, 0.0));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN3(1.0, 0.0));
	EXPECT_DOUBLE_EQ(1.0, tri->batozN3(0.0, 1.0));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN3(0.5, 0.5));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN3(0.0, 0.5));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN3(0.5, 0.0));

	// Shape function N4 weigth point 4 (mid-point 12) (parametric coordinate 0.5 0.5)
	EXPECT_DOUBLE_EQ(0.0, tri->batozN4(0.0, 0.0));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN4(1.0, 0.0));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN4(0.0, 1.0));
	EXPECT_DOUBLE_EQ(1.0, tri->batozN4(0.5, 0.5));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN4(0.0, 0.5));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN4(0.5, 0.0));

	// Shape function N5 weigth point 5 (mid-point 20) (parametric coordinate 0.0 0.5)
	EXPECT_DOUBLE_EQ(0.0, tri->batozN5(0.0, 0.0));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN5(1.0, 0.0));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN5(0.0, 1.0));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN5(0.5, 0.5));
	EXPECT_DOUBLE_EQ(1.0, tri->batozN5(0.0, 0.5));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN5(0.5, 0.0));

	// Shape function N6 weigth point 6 (mid-point 01) (parametric coordinate 0.5 0.0)
	EXPECT_DOUBLE_EQ(0.0, tri->batozN6(0.0, 0.0));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN6(1.0, 0.0));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN6(0.0, 1.0));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN6(0.5, 0.5));
	EXPECT_DOUBLE_EQ(0.0, tri->batozN6(0.0, 0.5));
	EXPECT_DOUBLE_EQ(1.0, tri->batozN6(0.5, 0.0));

	// We should have the relation sum(Ni(xi, neta) = 1) for all points in the triangle
	for (double xi = 0.0; xi <= 1.0; xi += 0.1)
	{
		for (double neta = 0.0; xi + neta <= 1.0; neta += 0.1)
		{
			EXPECT_DOUBLE_EQ(1.0, tri->batozN1(xi, neta) + tri->batozN2(xi, neta) + tri->batozN3(xi, neta) + \
				tri->batozN4(xi, neta) + tri->batozN5(xi, neta) + tri->batozN6(xi, neta)) <<
				"For (xi = " << xi << ", neta = " << neta << "), " << std::endl <<
				" N1 = " << tri->batozN1(xi, neta) << std::endl <<
				" N2 = " << tri->batozN2(xi, neta) << std::endl <<
				" N3 = " << tri->batozN3(xi, neta) << std::endl <<
				" N4 = " << tri->batozN4(xi, neta) << std::endl <<
				" N5 = " << tri->batozN5(xi, neta) << std::endl <<
				" N6 = " << tri->batozN6(xi, neta) << std::endl <<
				" N1+N2+N3+N4+N5+N6 = " <<
				tri->batozN1(xi, neta) + tri->batozN2(xi, neta) + tri->batozN3(xi, neta) +
				tri->batozN4(xi, neta) + tri->batozN5(xi, neta) + tri->batozN6(xi, neta);
		}
	}
}

TEST_F(Fem2DElementTriangleTests, StiffnessMatrixTest)
{
	std::shared_ptr<MockFem2DElement> tri = getElement();

	Eigen::Matrix<double, 18 ,18> expectedLocalStiffness;
	getExpectedLocalStiffnessMatrix(expectedLocalStiffness);
	EXPECT_TRUE(tri->getLocalStiffnessMatrix().isApprox(expectedLocalStiffness)) <<
		"KLocal = " << std::endl << tri->getLocalStiffnessMatrix() << std::endl <<
		"KLocal expected = " << std::endl << expectedLocalStiffness << std::endl;

	Eigen::Matrix<double, 18 ,18> R0 = tri->getInitialRotationTimes6();
	EXPECT_TRUE(tri->getGlobalStiffnessMatrix().isApprox(R0 * expectedLocalStiffness * R0.transpose())) <<
		"R0 = " << std::endl << R0 << std::endl <<
		"KGlobal = " << std::endl << tri->getLocalStiffnessMatrix() << std::endl <<
		"KGlobal expected = " << std::endl << expectedLocalStiffness << std::endl;
}

TEST_F(Fem2DElementTriangleTests, MassMatrixTest)
{
	std::shared_ptr<MockFem2DElement> tri = getElement();

	Eigen::Matrix<double, 18, 18> expectedMassMatrix;
	getExpectedLocalMassMatrix(expectedMassMatrix);
	EXPECT_TRUE(tri->getLocalMassMatrix().isApprox(expectedMassMatrix));

	Eigen::Matrix<double, 18 ,18> R0 = tri->getInitialRotationTimes6();
	EXPECT_TRUE(tri->getGlobalMassMatrix().isApprox(R0 * expectedMassMatrix * R0.transpose()));
}

TEST_F(Fem2DElementTriangleTests, ForceAndMatricesAPITest)
{
	using SurgSim::Math::addSubMatrix;

	std::shared_ptr<MockFem2DElement> tri = getElement();

	const size_t numDof = 6 * m_restState.getNumNodes();
	SurgSim::Math::Vector forceVector(numDof);
	SurgSim::Math::Vector ones(numDof);
	SurgSim::Math::Matrix massMatrix(numDof, numDof);
	SurgSim::Math::Matrix dampingMatrix(numDof, numDof);
	SurgSim::Math::Matrix stiffnessMatrix(numDof, numDof);
	SurgSim::Math::Matrix expectedMassMatrix(numDof, numDof);
	SurgSim::Math::Matrix expectedDampingMatrix(numDof, numDof);
	SurgSim::Math::Matrix expectedStiffnessMatrix(numDof, numDof);

	// Assemble manually the expectedStiffnessMatrix
	Eigen::Matrix<double, 18 ,18> R0 = tri->getInitialRotationTimes6();
	Eigen::Matrix<double, 18, 18> expected18x18StiffnessMatrix;
	getExpectedLocalStiffnessMatrix(expected18x18StiffnessMatrix);
	expectedStiffnessMatrix.setZero();
	addSubMatrix(R0 * expected18x18StiffnessMatrix * R0.transpose(), tri->getNodeIds(), 6, &expectedStiffnessMatrix);

	// Assemble manually the expectedMassMatrix
	Eigen::Matrix<double, 18, 18> expected18x18MassMatrix;
	getExpectedLocalMassMatrix(expected18x18MassMatrix);
	expectedMassMatrix.setZero();
	addSubMatrix(R0 * expected18x18MassMatrix * R0.transpose(), tri->getNodeIds(), 6, &expectedMassMatrix);

	forceVector.setZero();
	massMatrix.setZero();
	dampingMatrix.setZero();
	stiffnessMatrix.setZero();

	// No force should be produced when in rest state (x = x0) => F = K.(x-x0) = 0
	tri->addForce(m_restState, &forceVector);
	EXPECT_TRUE(forceVector.isZero());

	tri->addMass(m_restState, &massMatrix);
	EXPECT_TRUE(massMatrix.isApprox(expectedMassMatrix));

	tri->addDamping(m_restState, &dampingMatrix);
	EXPECT_TRUE(dampingMatrix.isZero());

	tri->addStiffness(m_restState, &stiffnessMatrix);
	EXPECT_TRUE(stiffnessMatrix.isApprox(expectedStiffnessMatrix));

	forceVector.setZero();
	massMatrix.setZero();
	dampingMatrix.setZero();
	stiffnessMatrix.setZero();

	tri->addFMDK(m_restState, &forceVector, &massMatrix, &dampingMatrix, &stiffnessMatrix);
	EXPECT_TRUE(forceVector.isZero());
	EXPECT_TRUE(massMatrix.isApprox(expectedMassMatrix));
	EXPECT_TRUE(dampingMatrix.isZero());
	EXPECT_TRUE(stiffnessMatrix.isApprox(expectedStiffnessMatrix));

	// Test addMatVec API with Mass component only
	forceVector.setZero();
	ones.setOnes();
	tri->addMatVec(m_restState, 1.0, 0.0, 0.0, ones, &forceVector);
	for (size_t rowId = 0; rowId < numDof; rowId++)
	{
		SCOPED_TRACE("Test addMatVec API with Mass component only");
		EXPECT_NEAR(expectedMassMatrix.row(rowId).sum(), forceVector[rowId], epsilon);
	}
	// Test addMatVec API with Damping component only
	forceVector.setZero();
	tri->addMatVec(m_restState, 0.0, 1.0, 0.0, ones, &forceVector);
	for (size_t rowId = 0; rowId < numDof; rowId++)
	{
		SCOPED_TRACE("Test addMatVec API with Damping component only");
		EXPECT_NEAR(0.0, forceVector[rowId], epsilon);
	}
	// Test addMatVec API with Stiffness component only
	forceVector.setZero();
	tri->addMatVec(m_restState, 0.0, 0.0, 1.0, ones, &forceVector);
	for (size_t rowId = 0; rowId < numDof; rowId++)
	{
		SCOPED_TRACE("Test addMatVec API with Stiffness component only");
		EXPECT_NEAR(expectedStiffnessMatrix.row(rowId).sum(), forceVector[rowId], epsilon);
	}
	// Test addMatVec API with mix Mass/Damping/Stiffness components
	forceVector.setZero();
	tri->addMatVec(m_restState, 1.0, 2.0, 3.0, ones, &forceVector);
	for (size_t rowId = 0; rowId < numDof; rowId++)
	{
		SCOPED_TRACE("Test addMatVec API with mix Mass/Damping/Stiffness components");
		double expectedCoef = 1.0 * expectedMassMatrix.row(rowId).sum() +
			3.0 * expectedStiffnessMatrix.row(rowId).sum();
		EXPECT_NEAR(expectedCoef, forceVector[rowId], epsilon * 10);
	}
}
