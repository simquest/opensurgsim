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

#include <SurgSim/Physics/Actors/Shapes/Shapes.h>

#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

using namespace SurgSim::Physics;

using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::RigidTransform3d;

class RigidShapeTest : public ::testing::Test
{
public:
	void SetUp()
	{
		m_rho = 9000.0;
		m_radius = 0.01;
		m_innerRadius = 0.005;
		m_length = 0.1;
		m_size[0] = 0.1;
		m_size[1] = 0.2;
		m_size[2] = 0.3;
	}

	void TearDown()
	{
	}

	// Mass density
	double m_rho;

	// Radius (sphere/cylinder/capsule)
	double m_radius;

	// Inner radius (cylinder)
	double m_innerRadius;

	// Length (cylinder/capsule)
	double m_length;

	// Size (box)
	double m_size[3];
};

TEST_F(RigidShapeTest, Sphere)
{
	ASSERT_NO_THROW({SphereShape s(m_radius);});

	{
		SphereShape s(m_radius, true);
		EXPECT_EQ(m_radius, s.getRadius());
		EXPECT_EQ(true, s.isSolid());

		const double& r = m_radius;
		const double r2 = r * r;
		double expectedMass = m_rho * 4.0 / 3.0 * M_PI * (r2 * r);
		double coef = 2.0 / 5.0 * expectedMass * r2;
		Matrix33d expectedInertia;
		expectedInertia << coef, 0.0, 0.0,
			0.0, coef, 0.0,
			0.0, 0.0, coef;

		double volume, mass;
		Vector3d massCenter;
		Matrix33d inertia;
		volume     = s.calculateVolume();
		mass       = s.calculateMass(m_rho);
		massCenter = s.calculateMassCenter();
		inertia    = s.calculateInertia(m_rho);

		
		double epsilon = 1e-10;
		EXPECT_NEAR(expectedMass, mass, epsilon);
		EXPECT_TRUE(expectedInertia.isApprox(inertia));
	}

	{
		SphereShape s(m_radius, false);
		EXPECT_EQ(m_radius, s.getRadius());
		EXPECT_EQ(false, s.isSolid());

		double expectedMass = m_rho * 4.0 * M_PI * (m_radius * m_radius);
		double coef = 2.0 / 3.0 * expectedMass * m_radius * m_radius;
		Matrix33d expectedInertia;
		expectedInertia << coef, 0.0, 0.0,
			0.0, coef, 0.0,
			0.0, 0.0, coef;

		double volume, mass;
		Vector3d massCenter;
		Matrix33d inertia;
		volume     = s.calculateVolume();
		mass       = s.calculateMass(m_rho);
		massCenter = s.calculateMassCenter();
		inertia    = s.calculateInertia(m_rho);
		
		double epsilon = 1e-10;
		EXPECT_NEAR(expectedMass, mass, epsilon);
		EXPECT_TRUE(expectedInertia.isApprox(inertia));
	}
}

TEST_F(RigidShapeTest, Box)
{
	ASSERT_NO_THROW({BoxShape b(m_size[0], m_size[1], m_size[2]);});

	BoxShape b(m_size[0], m_size[1], m_size[2]);
	EXPECT_EQ(m_size[0], b.getSizeX());
	EXPECT_EQ(m_size[1], b.getSizeY());
	EXPECT_EQ(m_size[2], b.getSizeZ());

	double expectedMass = m_rho * (m_size[0] * m_size[1] * m_size[2]);
	double coef = 1.0 / 12.0 * expectedMass;
	double x2 = m_size[0] * m_size[0];
	double y2 = m_size[1] * m_size[1];
	double z2 = m_size[2] * m_size[2];
	Matrix33d expectedInertia;
	expectedInertia << coef*(y2 + z2), 0.0, 0.0,
		0.0, coef*(x2 + z2), 0.0,
		0.0, 0.0, coef*(x2 + y2);

	double volume, mass;
	Vector3d massCenter;
	Matrix33d inertia;
	volume     = b.calculateVolume();
	mass       = b.calculateMass(m_rho);
	massCenter = b.calculateMassCenter();
	inertia    = b.calculateInertia(m_rho);
		
	double epsilon = 1e-10;
	EXPECT_NEAR(expectedMass, mass, epsilon);
	EXPECT_TRUE(expectedInertia.isApprox(inertia));
}

TEST_F(RigidShapeTest, CylinderX)
{
	{
		ASSERT_NO_THROW({CylinderShape<SHAPE_DIRECTION_AXIS_X> c(m_length, m_radius);});

		CylinderShape<SHAPE_DIRECTION_AXIS_X> c(m_length, m_radius);
		EXPECT_EQ(m_length, c.getLength());
		EXPECT_EQ(0.0, c.getInnerRadius());
		EXPECT_EQ(m_radius, c.getOuterRadius());

		double expectedMass = m_rho * (M_PI * m_radius * m_radius * m_length);

		double r1sq = m_radius * m_radius;
		double l2 = m_length * m_length;
		double coefDir = 1.0 /  2.0 * expectedMass * (r1sq);
		double coef    = 1.0 / 12.0 * expectedMass * (3.0 * (r1sq) + l2);
		Matrix33d expectedInertia;
		expectedInertia << coefDir, 0.0, 0.0,
			0.0, coef, 0.0,
			0.0, 0.0, coef;

		double volume, mass;
		Vector3d massCenter;
		Matrix33d inertia;
		volume     = c.calculateVolume();
		mass       = c.calculateMass(m_rho);
		massCenter = c.calculateMassCenter();
		inertia    = c.calculateInertia(m_rho);

		double epsilon = 1e-10;
		EXPECT_NEAR(expectedMass, mass, epsilon);
		EXPECT_TRUE(expectedInertia.isApprox(inertia));
	}

	{
		ASSERT_NO_THROW({CylinderShape<SHAPE_DIRECTION_AXIS_X> c(m_length, m_radius, m_innerRadius);});

		CylinderShape<SHAPE_DIRECTION_AXIS_X> c(m_length, m_radius, m_innerRadius);
		EXPECT_EQ(m_length, c.getLength());
		EXPECT_EQ(m_innerRadius, c.getInnerRadius());
		EXPECT_EQ(m_radius, c.getOuterRadius());

		double r1sq = m_radius * m_radius;
		double r2sq = m_innerRadius * m_innerRadius;
		double l2 = m_length * m_length;

		double expectedMass = m_rho * M_PI * (r1sq - r2sq) * m_length;
		double coefDir = 1.0 /  2.0 * expectedMass * (r1sq + r2sq);
		double coef    = 1.0 / 12.0 * expectedMass * (3.0 * (r1sq + r2sq) + l2);
		Matrix33d expectedInertia;
		expectedInertia << coefDir, 0.0, 0.0,
			0.0, coef, 0.0,
			0.0, 0.0, coef;

		double volume, mass;
		Vector3d massCenter;
		Matrix33d inertia;
		volume     = c.calculateVolume();
		mass       = c.calculateMass(m_rho);
		massCenter = c.calculateMassCenter();
		inertia    = c.calculateInertia(m_rho);

		double epsilon = 1e-10;
		EXPECT_NEAR(expectedMass, mass, epsilon);
		EXPECT_TRUE(expectedInertia.isApprox(inertia));
	}
}

TEST_F(RigidShapeTest, CylinderY)
{
	{
		ASSERT_NO_THROW({CylinderShape<SHAPE_DIRECTION_AXIS_Y> c(m_length, m_radius);});

		CylinderShape<SHAPE_DIRECTION_AXIS_Y> c(m_length, m_radius);
		EXPECT_EQ(m_length, c.getLength());
		EXPECT_EQ(0.0, c.getInnerRadius());
		EXPECT_EQ(m_radius, c.getOuterRadius());

		double expectedMass = m_rho * (M_PI * m_radius * m_radius * m_length);

		double r1sq = m_radius * m_radius;
		double l2 = m_length * m_length;
		double coefDir = 1.0 /  2.0 * expectedMass * (r1sq);
		double coef    = 1.0 / 12.0 * expectedMass * (3.0 * (r1sq) + l2);
		Matrix33d expectedInertia;
		expectedInertia << coef, 0.0, 0.0,
			0.0, coefDir, 0.0,
			0.0, 0.0, coef;

		double volume, mass;
		Vector3d massCenter;
		Matrix33d inertia;
		volume     = c.calculateVolume();
		mass       = c.calculateMass(m_rho);
		massCenter = c.calculateMassCenter();
		inertia    = c.calculateInertia(m_rho);

		double epsilon = 1e-10;
		EXPECT_NEAR(expectedMass, mass, epsilon);
		EXPECT_TRUE(expectedInertia.isApprox(inertia));
	}

	{
		ASSERT_NO_THROW({CylinderShape<SHAPE_DIRECTION_AXIS_Y> c(m_length, m_radius, m_innerRadius);});

		CylinderShape<SHAPE_DIRECTION_AXIS_Y> c(m_length, m_radius, m_innerRadius);
		EXPECT_EQ(m_length, c.getLength());
		EXPECT_EQ(m_innerRadius, c.getInnerRadius());
		EXPECT_EQ(m_radius, c.getOuterRadius());

		double r1sq = m_radius * m_radius;
		double r2sq = m_innerRadius * m_innerRadius;
		double l2 = m_length * m_length;

		double expectedMass = m_rho * M_PI * (r1sq - r2sq) * m_length;
		double coefDir = 1.0 /  2.0 * expectedMass * (r1sq + r2sq);
		double coef    = 1.0 / 12.0 * expectedMass * (3.0 * (r1sq + r2sq) + l2);
		Matrix33d expectedInertia;
		expectedInertia << coef, 0.0, 0.0,
			0.0, coefDir, 0.0,
			0.0, 0.0, coef;

		double volume, mass;
		Vector3d massCenter;
		Matrix33d inertia;
		volume     = c.calculateVolume();
		mass       = c.calculateMass(m_rho);
		massCenter = c.calculateMassCenter();
		inertia    = c.calculateInertia(m_rho);

		double epsilon = 1e-10;
		EXPECT_NEAR(expectedMass, mass, epsilon);
		EXPECT_TRUE(expectedInertia.isApprox(inertia));
	}
}

TEST_F(RigidShapeTest, CylinderZ)
{
	{
		ASSERT_NO_THROW({CylinderShape<SHAPE_DIRECTION_AXIS_Z> c(m_length, m_radius);});

		CylinderShape<SHAPE_DIRECTION_AXIS_Z> c(m_length, m_radius);
		EXPECT_EQ(m_length, c.getLength());
		EXPECT_EQ(0.0, c.getInnerRadius());
		EXPECT_EQ(m_radius, c.getOuterRadius());

		double expectedMass = m_rho * (M_PI * m_radius * m_radius * m_length);

		double r1sq = m_radius * m_radius;
		double l2 = m_length * m_length;
		double coefDir = 1.0 /  2.0 * expectedMass * (r1sq);
		double coef    = 1.0 / 12.0 * expectedMass * (3.0 * (r1sq) + l2);
		Matrix33d expectedInertia;
		expectedInertia << coef, 0.0, 0.0,
			0.0, coef, 0.0,
			0.0, 0.0, coefDir;

		double volume, mass;
		Vector3d massCenter;
		Matrix33d inertia;
		volume     = c.calculateVolume();
		mass       = c.calculateMass(m_rho);
		massCenter = c.calculateMassCenter();
		inertia    = c.calculateInertia(m_rho);

		double epsilon = 1e-10;
		EXPECT_NEAR(expectedMass, mass, epsilon);
		EXPECT_TRUE(expectedInertia.isApprox(inertia));
	}

	{
		ASSERT_NO_THROW({CylinderShape<SHAPE_DIRECTION_AXIS_Z> c(m_length, m_radius, m_innerRadius);});

		CylinderShape<SHAPE_DIRECTION_AXIS_Z> c(m_length, m_radius, m_innerRadius);
		EXPECT_EQ(m_length, c.getLength());
		EXPECT_EQ(m_innerRadius, c.getInnerRadius());
		EXPECT_EQ(m_radius, c.getOuterRadius());

		double r1sq = m_radius * m_radius;
		double r2sq = m_innerRadius * m_innerRadius;
		double l2 = m_length * m_length;

		double expectedMass = m_rho * M_PI * (r1sq - r2sq) * m_length;
		double coefDir = 1.0 /  2.0 * expectedMass * (r1sq + r2sq);
		double coef    = 1.0 / 12.0 * expectedMass * (3.0 * (r1sq + r2sq) + l2);
		Matrix33d expectedInertia;
		expectedInertia << coef, 0.0, 0.0,
			0.0, coef, 0.0,
			0.0, 0.0, coefDir;

		double volume, mass;
		Vector3d massCenter;
		Matrix33d inertia;
		volume     = c.calculateVolume();
		mass       = c.calculateMass(m_rho);
		massCenter = c.calculateMassCenter();
		inertia    = c.calculateInertia(m_rho);

		double epsilon = 1e-10;
		EXPECT_NEAR(expectedMass, mass, epsilon);
		EXPECT_TRUE(expectedInertia.isApprox(inertia));
	}
}

TEST_F(RigidShapeTest, CapsuleX)
{
	ASSERT_NO_THROW({CapsuleShape<SHAPE_DIRECTION_AXIS_X> c(m_length, m_radius);});

	CapsuleShape<SHAPE_DIRECTION_AXIS_X> c(m_length, m_radius);
	EXPECT_EQ(m_length, c.getLength());
	EXPECT_EQ(m_radius, c.getRadius());

	double r2 = m_radius * m_radius;
	double r3 = r2 * m_radius;
	double l2 = m_length * m_length;

	double massCylinder = m_rho * (M_PI * r2 * m_length);
	double massSphere = m_rho * (4.0 / 3.0 * M_PI * r3);
	double expectedMass = massCylinder + massSphere;
	double coefDir = 2.0 /  5.0 * massSphere * r2;
	double coef    = coefDir;
	coefDir += 1.0 / 2.0 * massCylinder * r2;
	coef += massSphere  * (1.0 / 4.0 * l2 + 3.0 / 8.0 * m_radius * m_length);
	coef += 1.0 / 12.0 * massCylinder * (3*r2 + l2);
	Matrix33d expectedInertia;
	expectedInertia << coefDir, 0.0, 0.0,
		0.0, coef, 0.0,
		0.0, 0.0, coef;

	double volume, mass;
	Vector3d massCenter;
	Matrix33d inertia;
	volume     = c.calculateVolume();
	mass       = c.calculateMass(m_rho);
	massCenter = c.calculateMassCenter();
	inertia    = c.calculateInertia(m_rho);

	double epsilon = 1e-10;
	EXPECT_NEAR(expectedMass, mass, epsilon);
	EXPECT_TRUE(expectedInertia.isApprox(inertia));
}

TEST_F(RigidShapeTest, CapsuleY)
{
	ASSERT_NO_THROW({CapsuleShape<SHAPE_DIRECTION_AXIS_Y> c(m_length, m_radius);});

	CapsuleShape<SHAPE_DIRECTION_AXIS_Y> c(m_length, m_radius);
	EXPECT_EQ(m_length, c.getLength());
	EXPECT_EQ(m_radius, c.getRadius());

	double r2 = m_radius * m_radius;
	double r3 = r2 * m_radius;
	double l2 = m_length * m_length;

	double massCylinder = m_rho * (M_PI * r2 * m_length);
	double massSphere = m_rho * (4.0 / 3.0 * M_PI * r3);
	double expectedMass = massCylinder + massSphere;
	double coefDir = 2.0 /  5.0 * massSphere * r2;
	double coef    = coefDir;
	coefDir += 1.0 / 2.0 * massCylinder * r2;
	coef += massSphere  * (1.0 / 4.0 * l2 + 3.0 / 8.0 * m_radius * m_length);
	coef += 1.0 / 12.0 * massCylinder * (3*r2 + l2);
	Matrix33d expectedInertia;
	expectedInertia << coef, 0.0, 0.0,
		0.0, coefDir, 0.0,
		0.0, 0.0, coef;

	double volume, mass;
	Vector3d massCenter;
	Matrix33d inertia;
	volume     = c.calculateVolume();
	mass       = c.calculateMass(m_rho);
	massCenter = c.calculateMassCenter();
	inertia    = c.calculateInertia(m_rho);

	double epsilon = 1e-10;
	EXPECT_NEAR(expectedMass, mass, epsilon);
	EXPECT_TRUE(expectedInertia.isApprox(inertia));
}

TEST_F(RigidShapeTest, CapsuleZ)
{
	ASSERT_NO_THROW({CapsuleShape<SHAPE_DIRECTION_AXIS_Z> c(m_length, m_radius);});

	CapsuleShape<SHAPE_DIRECTION_AXIS_Z> c(m_length, m_radius);
	EXPECT_EQ(m_length, c.getLength());
	EXPECT_EQ(m_radius, c.getRadius());

	double r2 = m_radius * m_radius;
	double r3 = r2 * m_radius;
	double l2 = m_length * m_length;

	double massCylinder = m_rho * (M_PI * r2 * m_length);
	double massSphere = m_rho * (4.0 / 3.0 * M_PI * r3);
	double expectedMass = massCylinder + massSphere;
	double coefDir = 2.0 /  5.0 * massSphere * r2;
	double coef    = coefDir;
	coefDir += 1.0 / 2.0 * massCylinder * r2;
	coef += massSphere  * (1.0 / 4.0 * l2 + 3.0 / 8.0 * m_radius * m_length);
	coef += 1.0 / 12.0 * massCylinder * (3*r2 + l2);
	Matrix33d expectedInertia;
	expectedInertia << coef, 0.0, 0.0,
		0.0, coef, 0.0,
		0.0, 0.0, coefDir;

	double volume, mass;
	Vector3d massCenter;
	Matrix33d inertia;
	volume     = c.calculateVolume();
	mass       = c.calculateMass(m_rho);
	massCenter = c.calculateMassCenter();
	inertia    = c.calculateInertia(m_rho);

	double epsilon = 1e-10;
	EXPECT_NEAR(expectedMass, mass, epsilon);
	EXPECT_TRUE(expectedInertia.isApprox(inertia));
}
