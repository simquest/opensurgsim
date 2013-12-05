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

#include <string>

#include "SurgSim/Physics/LinearSpring.h"
#include "SurgSim/Physics/DeformableRepresentationState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"

using SurgSim::Physics::LinearSpring;
using SurgSim::Physics::DeformableRepresentationState;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix;
using SurgSim::Math::Matrix33d;

TEST(LinearSpringTests, Constructor)
{
	ASSERT_NO_THROW({LinearSpring ls(0, 1);});
	ASSERT_NO_THROW({LinearSpring *ls = new LinearSpring(0, 1); delete ls;});
	ASSERT_NO_THROW({std::shared_ptr<LinearSpring> ls = std::make_shared<LinearSpring>(0, 1);});
}

TEST(LinearSpringTests, SetGetMethods)
{
	LinearSpring ls(0, 1);

	// Stiffness getter/setter
	ls.setStiffness(0.34);
	ASSERT_DOUBLE_EQ(0.34, ls.getStiffness());

	// Damping getter/setter
	ls.setDamping(0.45);
	ASSERT_DOUBLE_EQ(0.45, ls.getDamping());

	// Rest length getter/setter
	ls.setRestLength(1.23);
	ASSERT_DOUBLE_EQ(1.23, ls.getRestLength());

	// Operator ==/!= (with same node Ids)
	LinearSpring ls2(0, 1);
	ASSERT_TRUE(ls != ls2);
	ls2.setStiffness(ls.getStiffness());
	ASSERT_TRUE(ls != ls2);
	ls2.setDamping(ls.getDamping());
	ASSERT_TRUE(ls != ls2);
	ls2.setRestLength(ls.getRestLength());
	ASSERT_TRUE(ls == ls2);
	ls2.setDamping(ls.getDamping() + 0.55);
	ASSERT_TRUE(ls != ls2);
	ls2.setDamping(ls.getDamping());
	ls2.setStiffness(ls.getStiffness() + 0.23);
	ASSERT_TRUE(ls != ls2);

	// Operator ==/!= (with different node Ids)
	LinearSpring ls3(0, 2);
	ASSERT_TRUE(ls != ls3);
	ls3.setStiffness(ls.getStiffness());
	ASSERT_TRUE(ls != ls3);
	ls3.setDamping(ls.getDamping());
	ASSERT_TRUE(ls != ls3);
	ls3.setRestLength(ls.getRestLength());
	ASSERT_TRUE(ls != ls3);
}

TEST(LinearSpringTests, computeMethods)
{
	using SurgSim::Math::setSubVector;
	using SurgSim::Math::setSubMatrix;

	LinearSpring ls(0, 1);
	ls.setStiffness(0.34);
	ls.setDamping(0.45);
	ls.setRestLength(1.23);

	// Calculating spring force
	Vector3d expectedF3D(0.58375208191171812, 1.0406015373208888, 0.30456630360611381);
	DeformableRepresentationState state;
	Vector expectedF;
	state.setNumDof(3u, 2u);
	expectedF.resize(6u);
	setSubVector(Vector3d(0.0, 0.0, 0.0), 0, 3, &state.getPositions());
	setSubVector(Vector3d(2.3, 4.1, 1.2), 1, 3, &state.getPositions());
	setSubVector(expectedF3D, 0, 3, &expectedF);
	setSubVector(-expectedF3D, 1, 3, &expectedF);
	const Vector& f = ls.computeForce(state);
	EXPECT_TRUE(f.isApprox(expectedF));

	// Calculate stiffness matrix
	double expectedStiffnessMatrixContent[] = {
		0.27317527049035600, 0.034529161604161258, 0.010106096079266710,
		0.034529161604161258, 0.31535723673425187, 0.018015214749997177,
		0.010106096079266710, 0.018015214749997177, 0.25907799878558180
	};
	Matrix33d expectedStiffness33(expectedStiffnessMatrixContent);
	Matrix expectedK;
	expectedK.resize(6u, 6u);
	setSubMatrix( expectedStiffness33, 0, 0, 3, 3, &expectedK);
	setSubMatrix(-expectedStiffness33, 0, 1, 3, 3, &expectedK);
	setSubMatrix(-expectedStiffness33, 1, 0, 3, 3, &expectedK);
	setSubMatrix( expectedStiffness33, 1, 1, 3, 3, &expectedK);
	const Matrix& K = ls.computeStiffness(state);
	EXPECT_TRUE(K.isApprox(expectedK));

	// Calculate damping matrix
	const Matrix& D = ls.computeDamping(state);
	EXPECT_TRUE(D.isZero());

	// Compute all together
	{
		Vector *f;
		Matrix *K, *D;
		ls.computeFDK(state, &f, &D, &K);
		EXPECT_TRUE(f->isApprox(expectedF));
		EXPECT_TRUE(K->isApprox(expectedK));
		EXPECT_TRUE(D->isZero());
	}
}
