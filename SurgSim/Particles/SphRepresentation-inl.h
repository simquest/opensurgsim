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

#ifndef SURGSIM_PARTICLES_SPHREPRESENTATION_INL_H
#define SURGSIM_PARTICLES_SPHREPRESENTATION_INL_H

namespace SurgSim
{

namespace Particles
{

template <typename Derived>
double SphRepresentation::KernelPoly6(const Eigen::MatrixBase<Derived>& rij)
{
	double rPow2 = rij.squaredNorm();

	if (rPow2 <= m_hPow2)
	{
		double hPow2_minus_rPow2 = (m_hPow2 - rPow2);
		return 315.0 / (64.0 * M_PI * m_hPow9) * hPow2_minus_rPow2 * hPow2_minus_rPow2 * hPow2_minus_rPow2;
	}
	else
	{
		return 0.0;
	}
}

template <typename Derived>
SurgSim::Math::Vector3d SphRepresentation::KernelPoly6Gradient(const Eigen::MatrixBase<Derived>& rij)
{
	double rPow2 = rij.squaredNorm();

	if (rPow2 <= m_hPow2)
	{
		double hPow2_minus_rPow2 = (m_hPow2 - rPow2);
		return -rij * 945.0 / (32.0 * M_PI * m_hPow9) * hPow2_minus_rPow2 * hPow2_minus_rPow2;
	}
	else
	{
		return SurgSim::Math::Vector3d::Zero();
	}
}

template <typename Derived>
double SphRepresentation::KernelPoly6Laplacian(const Eigen::MatrixBase<Derived>& rij)
{
	double rPow2 = rij.squaredNorm();

	if (rPow2 <= m_hPow2)
	{
		double hPow2_minus_rPow2 = (m_hPow2 - rPow2);
		return 945.0 / (8.0 * M_PI * m_hPow9) * hPow2_minus_rPow2 * (rPow2 - 3.0 / 4.0 * hPow2_minus_rPow2);
	}
	else
	{
		return 0.0;
	}
}

template <typename Derived>
double SphRepresentation::KernelSpiky(const Eigen::MatrixBase<Derived>& rij)
{
	double r = rij.norm();

	if (r <= this->m_h)
	{
		double h_minus_r = this->m_h - r;
		return 15.0 / (M_PI * m_hPow6) * h_minus_r * h_minus_r * h_minus_r;
	}
	else
	{
		return 0.0;
	}
}

template <typename Derived>
SurgSim::Math::Vector3d SphRepresentation::KernelSpikyGradient(const Eigen::MatrixBase<Derived>& rij)
{
	double r = rij.norm();

	if (r <= this->m_h)
	{
		double h_minus_r = this->m_h - r;
		return -rij * 45.0 / (M_PI * m_hPow6 * r) * h_minus_r * h_minus_r;
	}
	else
	{
		return SurgSim::Math::Vector3d::Zero();
	}
}

template <typename Derived>
double SphRepresentation::KernelViscosity(const Eigen::MatrixBase<Derived>& rij)
{
	double r = rij.norm();

	if (r <= this->m_h)
	{
		double q = r / this->m_h;
		double qPow2 = q * q;
		return 15.0 / (2.0 * M_PI * m_hPow3) * (-0.5 * q * qPow2 + qPow2 + 0.5 / q - 1.0);
	}
	else
	{
		return 0.0;
	}
}

template <typename Derived>
SurgSim::Math::Vector3d SphRepresentation::KernelViscosityGradient(const Eigen::MatrixBase<Derived>& rij)
{
	double r = rij.norm();

	if (r <= this->m_h)
	{
		return rij * 15.0 / (2.0 * M_PI * m_hPow3) * (-1.5 * r / m_hPow3 + 2.0 / m_hPow2 - 0.5 * this->m_h / (r*r*r));
	}
	else
	{
		return SurgSim::Math::Vector3d::Zero();
	}
}

template <typename Derived>
double SphRepresentation::KernelViscosityLaplacian(const Eigen::MatrixBase<Derived>& rij)
{
	double r = rij.norm();

	if (r <= this->m_h)
	{
		return 45.0 / (M_PI * m_hPow5) * (1.0 - r / this->m_h);
	}
	else
	{
		return 0.0;
	}
}

};  // namespace Particles

};  // namespace SurgSim

#endif  // SURGSIM_PARTICLES_SPHREPRESENTATION_INL_H
