//// This file is a part of the OpenSurgSim project.
//// Copyright 2013, SimQuest Solutions Inc.
////
//// Licensed under the Apache License, Version 2.0 (the "License");
//// you may not use this file except in compliance with the License.
//// You may obtain a copy of the License at
////
////     http://www.apache.org/licenses/LICENSE-2.0
////
//// Unless required by applicable law or agreed to in writing, software
//// distributed under the License is distributed on an "AS IS" BASIS,
//// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//// See the License for the specific language governing permissions and
//// limitations under the License.

#include <gtest/gtest.h>

#include <string>

#include <SurgSim/Physics/Actors/RigidShape.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

using namespace SurgSim::Physics;

using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector3d;
using SurgSim::Math::RigidTransform3d;

class RigidShapeTest : public ::testing::Test
{
public:
	void SetUp()
	{
		rho = 9000;
		radius = 0.01;
		innerRadius = 0.005;
		length = 0.1;
		size[0] = 0.1;
		size[1] = 0.2;
		size[2] = 0.3;
	}

	void TearDown()
	{
	}

	// Mass density
	double rho;

	// Radius (sphere/cylinder/capsule)
	double radius;

	// Inner radius (cylinder)
	double innerRadius;

	// Length (cylinder/capsule)
	double length;

	// Size (box)
	double size[3];
};

TEST_F(RigidShapeTest, Sphere)
{
	ASSERT_NO_THROW({SphereShape s(radius);});

	{
		SphereShape s(radius, true);
		EXPECT_EQ(radius, s.getRadius());
		EXPECT_EQ(true, s.isSolid());

		const double& r = radius;
		const double r2 = r * r;
		double expectedMass = rho * 4.0 / 3.0 * M_PI * (r2 * r);
		double coef = 2.0 / 5.0 * expectedMass * r2;
		Matrix33d expectedInertia;
		expectedInertia << coef, 0.0, 0.0, 0.0, coef, 0.0, 0.0, 0.0, coef;

		double volume, mass;
		Vector3d massCenter;
		Matrix33d inertia;
		s.calculateMassInertia(rho, &volume, &mass, &massCenter, &inertia);
		
		double epsilon = 1e-10;
		EXPECT_NEAR(expectedMass, mass, epsilon);
		EXPECT_TRUE(expectedInertia.isApprox(inertia));
	}

	{
		SphereShape s(radius, false);
		EXPECT_EQ(radius, s.getRadius());
		EXPECT_EQ(false, s.isSolid());

		double expectedMass = rho * 4.0 * M_PI * (radius * radius);
		double coef = 2.0 / 3.0 * expectedMass * radius * radius;
		Matrix33d expectedInertia;
		expectedInertia << coef, 0.0, 0.0,
			0.0, coef, 0.0,
			0.0, 0.0, coef;

		double volume, mass;
		Vector3d massCenter;
		Matrix33d inertia;
		s.calculateMassInertia(rho, &volume, &mass, &massCenter, &inertia);
		
		double epsilon = 1e-10;
		EXPECT_NEAR(expectedMass, mass, epsilon);
		EXPECT_TRUE(expectedInertia.isApprox(inertia));
	}
}

TEST_F(RigidShapeTest, Box)
{
	ASSERT_NO_THROW({BoxShape b(size[0], size[1], size[2]);});

	BoxShape b(size[0], size[1], size[2]);
	EXPECT_EQ(size[0], b.getSizeX());
	EXPECT_EQ(size[1], b.getSizeY());
	EXPECT_EQ(size[2], b.getSizeZ());

	double expectedMass = rho * (size[0] * size[1] * size[2]);
	double coef = 1.0 / 12.0 * expectedMass;
	double x2 = size[0] * size[0];
	double y2 = size[1] * size[1];
	double z2 = size[2] * size[2];
	Matrix33d expectedInertia;
	expectedInertia << coef*(y2 + z2), 0.0, 0.0,
		0.0, coef*(x2 + z2), 0.0,
		0.0, 0.0, coef*(x2 + y2);

	double volume, mass;
	Vector3d massCenter;
	Matrix33d inertia;
	b.calculateMassInertia(rho, &volume, &mass, &massCenter, &inertia);
		
	double epsilon = 1e-10;
	EXPECT_NEAR(expectedMass, mass, epsilon);
	EXPECT_TRUE(expectedInertia.isApprox(inertia));
}

TEST_F(RigidShapeTest, CylinderX)
{
	{
		ASSERT_NO_THROW({CylinderShape<0> c(length, radius);});

		CylinderShape<0> c(length, radius);
		EXPECT_EQ(length, c.getLength());
		EXPECT_EQ(0.0, c.getInnerRadius());
		EXPECT_EQ(radius, c.getOuterRadius());

		double expectedMass = rho * (M_PI * radius * radius * length);

		double r1sq = radius * radius;
		double l2 = length * length;
		double coefDir = 1.0 /  2.0 * expectedMass * (r1sq);
		double coef    = 1.0 / 12.0 * expectedMass * (3.0 * (r1sq) + l2);
		Matrix33d expectedInertia;
		expectedInertia << coefDir, 0.0, 0.0,
			0.0, coef, 0.0,
			0.0, 0.0, coef;

		double volume, mass;
		Vector3d massCenter;
		Matrix33d inertia;
		c.calculateMassInertia(rho, &volume, &mass, &massCenter, &inertia);

		double epsilon = 1e-10;
		EXPECT_NEAR(expectedMass, mass, epsilon);
		EXPECT_TRUE(expectedInertia.isApprox(inertia));
	}

	{
		ASSERT_NO_THROW({CylinderShape<0> c(length, radius, innerRadius);});

		CylinderShape<0> c(length, radius, innerRadius);
		EXPECT_EQ(length, c.getLength());
		EXPECT_EQ(innerRadius, c.getInnerRadius());
		EXPECT_EQ(radius, c.getOuterRadius());

		double r1sq = radius * radius;
		double r2sq = innerRadius * innerRadius;
		double l2 = length * length;

		double expectedMass = rho * M_PI * (r1sq - r2sq) * length;
		double coefDir = 1.0 /  2.0 * expectedMass * (r1sq + r2sq);
		double coef    = 1.0 / 12.0 * expectedMass * (3.0 * (r1sq + r2sq) + l2);
		Matrix33d expectedInertia;
		expectedInertia << coefDir, 0.0, 0.0,
			0.0, coef, 0.0,
			0.0, 0.0, coef;

		double volume, mass;
		Vector3d massCenter;
		Matrix33d inertia;
		c.calculateMassInertia(rho, &volume, &mass, &massCenter, &inertia);

		double epsilon = 1e-10;
		EXPECT_NEAR(expectedMass, mass, epsilon);
		EXPECT_TRUE(expectedInertia.isApprox(inertia));
	}
}

TEST_F(RigidShapeTest, CylinderY)
{
	{
		ASSERT_NO_THROW({CylinderShape<1> c(length, radius);});

		CylinderShape<1> c(length, radius);
		EXPECT_EQ(length, c.getLength());
		EXPECT_EQ(0.0, c.getInnerRadius());
		EXPECT_EQ(radius, c.getOuterRadius());

		double expectedMass = rho * (M_PI * radius * radius * length);

		double r1sq = radius * radius;
		double l2 = length * length;
		double coefDir = 1.0 /  2.0 * expectedMass * (r1sq);
		double coef    = 1.0 / 12.0 * expectedMass * (3.0 * (r1sq) + l2);
		Matrix33d expectedInertia;
		expectedInertia << coef, 0.0, 0.0,
			0.0, coefDir, 0.0,
			0.0, 0.0, coef;

		double volume, mass;
		Vector3d massCenter;
		Matrix33d inertia;
		c.calculateMassInertia(rho, &volume, &mass, &massCenter, &inertia);

		double epsilon = 1e-10;
		EXPECT_NEAR(expectedMass, mass, epsilon);
		EXPECT_TRUE(expectedInertia.isApprox(inertia));
	}

	{
		ASSERT_NO_THROW({CylinderShape<1> c(length, radius, innerRadius);});

		CylinderShape<1> c(length, radius, innerRadius);
		EXPECT_EQ(length, c.getLength());
		EXPECT_EQ(innerRadius, c.getInnerRadius());
		EXPECT_EQ(radius, c.getOuterRadius());

		double r1sq = radius * radius;
		double r2sq = innerRadius * innerRadius;
		double l2 = length * length;

		double expectedMass = rho * M_PI * (r1sq - r2sq) * length;
		double coefDir = 1.0 /  2.0 * expectedMass * (r1sq + r2sq);
		double coef    = 1.0 / 12.0 * expectedMass * (3.0 * (r1sq + r2sq) + l2);
		Matrix33d expectedInertia;
		expectedInertia << coef, 0.0, 0.0,
			0.0, coefDir, 0.0,
			0.0, 0.0, coef;

		double volume, mass;
		Vector3d massCenter;
		Matrix33d inertia;
		c.calculateMassInertia(rho, &volume, &mass, &massCenter, &inertia);

		double epsilon = 1e-10;
		EXPECT_NEAR(expectedMass, mass, epsilon);
		EXPECT_TRUE(expectedInertia.isApprox(inertia));
	}
}

TEST_F(RigidShapeTest, CylinderZ)
{
	{
		ASSERT_NO_THROW({CylinderShape<2> c(length, radius);});

		CylinderShape<2> c(length, radius);
		EXPECT_EQ(length, c.getLength());
		EXPECT_EQ(0.0, c.getInnerRadius());
		EXPECT_EQ(radius, c.getOuterRadius());

		double expectedMass = rho * (M_PI * radius * radius * length);

		double r1sq = radius * radius;
		double l2 = length * length;
		double coefDir = 1.0 /  2.0 * expectedMass * (r1sq);
		double coef    = 1.0 / 12.0 * expectedMass * (3.0 * (r1sq) + l2);
		Matrix33d expectedInertia;
		expectedInertia << coef, 0.0, 0.0,
			0.0, coef, 0.0,
			0.0, 0.0, coefDir;

		double volume, mass;
		Vector3d massCenter;
		Matrix33d inertia;
		c.calculateMassInertia(rho, &volume, &mass, &massCenter, &inertia);

		double epsilon = 1e-10;
		EXPECT_NEAR(expectedMass, mass, epsilon);
		EXPECT_TRUE(expectedInertia.isApprox(inertia));
	}

	{
		ASSERT_NO_THROW({CylinderShape<2> c(length, radius, innerRadius);});

		CylinderShape<2> c(length, radius, innerRadius);
		EXPECT_EQ(length, c.getLength());
		EXPECT_EQ(innerRadius, c.getInnerRadius());
		EXPECT_EQ(radius, c.getOuterRadius());

		double r1sq = radius * radius;
		double r2sq = innerRadius * innerRadius;
		double l2 = length * length;

		double expectedMass = rho * M_PI * (r1sq - r2sq) * length;
		double coefDir = 1.0 /  2.0 * expectedMass * (r1sq + r2sq);
		double coef    = 1.0 / 12.0 * expectedMass * (3.0 * (r1sq + r2sq) + l2);
		Matrix33d expectedInertia;
		expectedInertia << coef, 0.0, 0.0,
			0.0, coef, 0.0,
			0.0, 0.0, coefDir;

		double volume, mass;
		Vector3d massCenter;
		Matrix33d inertia;
		c.calculateMassInertia(rho, &volume, &mass, &massCenter, &inertia);

		double epsilon = 1e-10;
		EXPECT_NEAR(expectedMass, mass, epsilon);
		EXPECT_TRUE(expectedInertia.isApprox(inertia));
	}
}

TEST_F(RigidShapeTest, CapsuleX)
{
	ASSERT_NO_THROW({CapsuleShape<0> c(length, radius);});

	CapsuleShape<0> c(length, radius);
	EXPECT_EQ(length, c.getLength());
	EXPECT_EQ(radius, c.getRadius());

	double r2 = radius * radius;
	double r3 = r2 * radius;
	double l2 = length * length;

	double massCylinder = rho * (M_PI * r2 * length);
	double massSphere = rho * (4.0 / 3.0 * M_PI * r3);
	double expectedMass = massCylinder + massSphere;
	double coefDir = 2.0 /  5.0 * massSphere * r2;
	double coef    = coefDir;
	coefDir += 1.0 / 2.0 * massCylinder * r2;
	coef += massSphere  * (1.0 / 4.0 * l2 + 3.0 / 8.0 * radius*length);
	coef += 1.0 / 12.0 * massCylinder * (3*r2 + l2);
	Matrix33d expectedInertia;
	expectedInertia << coefDir, 0.0, 0.0,
		0.0, coef, 0.0,
		0.0, 0.0, coef;

	double volume, mass;
	Vector3d massCenter;
	Matrix33d inertia;
	c.calculateMassInertia(rho, &volume, &mass, &massCenter, &inertia);

	double epsilon = 1e-10;
	EXPECT_NEAR(expectedMass, mass, epsilon);
	EXPECT_TRUE(expectedInertia.isApprox(inertia));
}

TEST_F(RigidShapeTest, CapsuleY)
{
	ASSERT_NO_THROW({CapsuleShape<1> c(length, radius);});

	CapsuleShape<1> c(length, radius);
	EXPECT_EQ(length, c.getLength());
	EXPECT_EQ(radius, c.getRadius());

	double r2 = radius * radius;
	double r3 = r2 * radius;
	double l2 = length * length;

	double massCylinder = rho * (M_PI * r2 * length);
	double massSphere = rho * (4.0 / 3.0 * M_PI * r3);
	double expectedMass = massCylinder + massSphere;
	double coefDir = 2.0 /  5.0 * massSphere * r2;
	double coef    = coefDir;
	coefDir += 1.0 / 2.0 * massCylinder * r2;
	coef += massSphere  * (1.0 / 4.0 * l2 + 3.0 / 8.0 * radius*length);
	coef += 1.0 / 12.0 * massCylinder * (3*r2 + l2);
	Matrix33d expectedInertia;
	expectedInertia << coef, 0.0, 0.0,
		0.0, coefDir, 0.0,
		0.0, 0.0, coef;

	double volume, mass;
	Vector3d massCenter;
	Matrix33d inertia;
	c.calculateMassInertia(rho, &volume, &mass, &massCenter, &inertia);

	double epsilon = 1e-10;
	EXPECT_NEAR(expectedMass, mass, epsilon);
	EXPECT_TRUE(expectedInertia.isApprox(inertia));
}

TEST_F(RigidShapeTest, CapsuleZ)
{
	ASSERT_NO_THROW({CapsuleShape<2> c(length, radius);});

	CapsuleShape<2> c(length, radius);
	EXPECT_EQ(length, c.getLength());
	EXPECT_EQ(radius, c.getRadius());

	double r2 = radius * radius;
	double r3 = r2 * radius;
	double l2 = length * length;

	double massCylinder = rho * (M_PI * r2 * length);
	double massSphere = rho * (4.0 / 3.0 * M_PI * r3);
	double expectedMass = massCylinder + massSphere;
	double coefDir = 2.0 /  5.0 * massSphere * r2;
	double coef    = coefDir;
	coefDir += 1.0 / 2.0 * massCylinder * r2;
	coef += massSphere  * (1.0 / 4.0 * l2 + 3.0 / 8.0 * radius*length);
	coef += 1.0 / 12.0 * massCylinder * (3*r2 + l2);
	Matrix33d expectedInertia;
	expectedInertia << coef, 0.0, 0.0,
		0.0, coef, 0.0,
		0.0, 0.0, coefDir;

	double volume, mass;
	Vector3d massCenter;
	Matrix33d inertia;
	c.calculateMassInertia(rho, &volume, &mass, &massCenter, &inertia);

	double epsilon = 1e-10;
	EXPECT_NEAR(expectedMass, mass, epsilon);
	EXPECT_TRUE(expectedInertia.isApprox(inertia));
}
