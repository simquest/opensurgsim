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

#include <SurgSim/Physics/FemElement.h>
#include <SurgSim/Physics/DeformableRepresentationState.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>

using SurgSim::Physics::FemElement;
using SurgSim::Physics::DeformableRepresentationState;
using SurgSim::Math::Vector;
using SurgSim::Math::Matrix;

class MockFemElement : public FemElement
{
public:
	MockFemElement() : FemElement()
	{
		setNumDofPerNode(3);
	}

	void addNode(unsigned int nodeId)
	{
		this->m_nodeIds.push_back(nodeId);
	}

	virtual double getVolume(const DeformableRepresentationState& state) const override
	{ return 1; }
	virtual void addForce(const DeformableRepresentationState& state, Vector* F) override
	{}
	virtual void addMass(const DeformableRepresentationState& state, Matrix* M) override
	{}
	virtual void addDamping(const DeformableRepresentationState& state, Matrix* D) override
	{}
	virtual void addStiffness(const DeformableRepresentationState& state, Matrix* K) override
	{}
	virtual void addFMDK(const DeformableRepresentationState& state, Vector* f, Matrix* M,
		Matrix* D, Matrix* K) override
	{}
};

void testSize(const Vector& v, int expectedSize)
{
	EXPECT_EQ(expectedSize, static_cast<int>(v.size()));
}

void testSize(const Matrix& m, int expectedRows, int expectedCols)
{
	EXPECT_EQ(expectedRows, static_cast<int>(m.rows()));
	EXPECT_EQ(expectedCols, static_cast<int>(m.cols()));
}

TEST(FemElementTests, GetSetAddMethods)
{
	MockFemElement femElement;

	// Initial setup (numDofPerNode set), no nodes defined yet, density = 0
	EXPECT_EQ(3u, femElement.getNumDofPerNode());
	EXPECT_EQ(0u, femElement.getNumNodes());
	EXPECT_EQ(0, femElement.getNodeIds().size());
	EXPECT_DOUBLE_EQ(0.0, femElement.getMassDensity());
	EXPECT_DOUBLE_EQ(0.0, femElement.getYoungModulus());
	EXPECT_DOUBLE_EQ(0.0, femElement.getPoissonRatio());

	// Test Set/Get Young modulus
	femElement.setYoungModulus(4455.33);
	EXPECT_DOUBLE_EQ(4455.33, femElement.getYoungModulus());
	femElement.setYoungModulus(0.0);
	EXPECT_DOUBLE_EQ(0.0, femElement.getYoungModulus());

	// Test Set/Get Poisson ratio
	femElement.setPoissonRatio(0.45);
	EXPECT_DOUBLE_EQ(0.45, femElement.getPoissonRatio());
	femElement.setPoissonRatio(0.0);
	EXPECT_DOUBLE_EQ(0.0, femElement.getPoissonRatio());

	// Test Set/Get mass density
	femElement.setMassDensity(2343.13);
	EXPECT_DOUBLE_EQ(2343.13, femElement.getMassDensity());
	femElement.setMassDensity(0.0);
	EXPECT_DOUBLE_EQ(0.0, femElement.getMassDensity());

	// Test GetMass
	DeformableRepresentationState fakeState;
	femElement.setMassDensity(0.0);
	EXPECT_DOUBLE_EQ(0.0, femElement.getMass(fakeState));
	femElement.setMassDensity(1.14);
	EXPECT_DOUBLE_EQ(1.14, femElement.getMass(fakeState));
	femElement.setMassDensity(434.55);
	EXPECT_DOUBLE_EQ(434.55, femElement.getMass(fakeState));

	// Add 1 node
	femElement.addNode(0);
	EXPECT_EQ(3u, femElement.getNumDofPerNode());
	EXPECT_EQ(1u, femElement.getNumNodes());
	EXPECT_EQ(1, femElement.getNodeIds().size());
	EXPECT_EQ(0, femElement.getNodeIds()[0]);
	EXPECT_EQ(0, femElement.getNodeId(0));

	// Add 1 more node
	femElement.addNode(9);
	EXPECT_EQ(3u, femElement.getNumDofPerNode());
	EXPECT_EQ(2u, femElement.getNumNodes());
	EXPECT_EQ(2, femElement.getNodeIds().size());
	EXPECT_EQ(0, femElement.getNodeIds()[0]);
	EXPECT_EQ(0, femElement.getNodeId(0));
	EXPECT_EQ(9, femElement.getNodeIds()[1]);
	EXPECT_EQ(9, femElement.getNodeId(1));
}
