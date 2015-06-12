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

/// \file DeformableTestsUtility.h
/// Common functions for testing deformable compute/update functions.

#ifndef SURGSIM_PHYSICS_UNITTESTS_DEFORMABLETESTSUTILITY_INL_H
#define SURGSIM_PHYSICS_UNITTESTS_DEFORMABLETESTSUTILITY_INL_H

template <class T>
void SurgSim::Physics::testOdeEquationUpdate(std::shared_ptr<T> rep,
	const SurgSim::Math::OdeState& state, const SurgSim::Math::Vector& expectedF,
	const SurgSim::Math::Matrix& expectedM, const SurgSim::Math::Matrix& expectedD,
	const SurgSim::Math::Matrix& expectedK)
{
	using SurgSim::Math::OdeEquationUpdate;

	auto odeEquation = std::dynamic_pointer_cast<SurgSim::Math::OdeEquation>(rep);

	odeEquation->updateFMDK(state, OdeEquationUpdate::ODEEQUATIONUPDATE_F);
	EXPECT_NO_THROW(EXPECT_TRUE(odeEquation->getF().isApprox(expectedF)));
	odeEquation->updateFMDK(state, OdeEquationUpdate::ODEEQUATIONUPDATE_M);
	EXPECT_NO_THROW(EXPECT_TRUE(odeEquation->getM().isApprox(expectedM)));
	odeEquation->updateFMDK(state, OdeEquationUpdate::ODEEQUATIONUPDATE_D);
	EXPECT_NO_THROW(EXPECT_TRUE(odeEquation->getD().isApprox(expectedD)));
	odeEquation->updateFMDK(state, OdeEquationUpdate::ODEEQUATIONUPDATE_K);
	EXPECT_NO_THROW(EXPECT_TRUE(odeEquation->getK().isApprox(expectedK)));

	// Test combo method computeFMDK
	rep->clearFMDK();
	odeEquation->updateFMDK(state, OdeEquationUpdate::ODEEQUATIONUPDATE_FMDK);
	EXPECT_NO_THROW(EXPECT_TRUE(odeEquation->getF().isApprox(expectedF)));
	EXPECT_NO_THROW(EXPECT_TRUE(odeEquation->getM().isApprox(expectedM)));
	EXPECT_NO_THROW(EXPECT_TRUE(odeEquation->getD().isApprox(expectedD)));
	EXPECT_NO_THROW(EXPECT_TRUE(odeEquation->getK().isApprox(expectedK)));
}

#endif // SURGSIM_PHYSICS_UNITTESTS_DEFORMABLETESTSUTILITY_INL_H
