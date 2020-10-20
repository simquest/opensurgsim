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

#include "SurgSim/Math/OdeEquation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Framework/Log.h"

namespace SurgSim
{

namespace Math
{

const std::shared_ptr<OdeState> OdeEquation::getInitialState() const
{
	return m_initialState;
}

const Vector& OdeEquation::getF() const
{
	return m_f;
}

const SparseMatrix& OdeEquation::getM() const
{
	return m_M;
}

const SparseMatrix& OdeEquation::getD() const
{
	return m_D;
}

const SparseMatrix& OdeEquation::getK() const
{
	return m_K;
}

bool OdeEquation::hasF() const
{
	return (m_initState & static_cast<int>(ODEEQUATIONUPDATE_F)) != 0;
}

bool OdeEquation::hasM() const
{
	return (m_initState & static_cast<int>(ODEEQUATIONUPDATE_M)) != 0;
}

bool OdeEquation::hasK() const
{
	return (m_initState & static_cast<int>(ODEEQUATIONUPDATE_K)) != 0;
}

bool OdeEquation::hasD() const
{
	return (m_initState & static_cast<int>(ODEEQUATIONUPDATE_D)) != 0;
}

void OdeEquation::updateFMDK(const OdeState& state, int options)
{
	if (options == ODEEQUATIONUPDATE_FMDK)
	{
		computeFMDK(state);
		m_initState |= static_cast<int>(ODEEQUATIONUPDATE_FMDK);
	}
	else
	{
		if (options & ODEEQUATIONUPDATE_F)
		{
			computeF(state);
			m_initState |= static_cast<int>(ODEEQUATIONUPDATE_F);
		}

		if (options & ODEEQUATIONUPDATE_M)
		{
			computeM(state);
			m_initState |= static_cast<int>(ODEEQUATIONUPDATE_M);
		}

		if (options & ODEEQUATIONUPDATE_D)
		{
			computeD(state);
			m_initState |= static_cast<int>(ODEEQUATIONUPDATE_D);
		}

		if (options & ODEEQUATIONUPDATE_K)
		{
			computeK(state);
			m_initState |= static_cast<int>(ODEEQUATIONUPDATE_K);
		}
	}
}

}; // namespace Math

}; // namespace SurgSim
