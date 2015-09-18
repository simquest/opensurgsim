// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

/// \file
/// Tests that exercise the functionality of our quaternion typedefs, which
/// come straight from Eigen.

#include <math.h>
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/MathConvert.h"
#include "gtest/gtest.h"

// Define test fixture class templates.
// We don't really need fixtures as such, but the templatization encodes type.

template <class T>
class QuaternionTests : public testing::Test
{
public:
	typedef T Quaternion;
	typedef typename T::AngleAxisType AngleAxis;
	typedef typename T::Scalar Scalar;
};

// This used to contain aligned (via Eigen::AutoAlign) quaternion type aliases, but we got rid of those.
typedef ::testing::Types<SurgSim::Math::Quaterniond,
		SurgSim::Math::Quaternionf> QuaternionVariants;
TYPED_TEST_CASE(QuaternionTests, QuaternionVariants);



template <class T>
class UnalignedQuaternionTests : public QuaternionTests<T>
{
};

typedef ::testing::Types<SurgSim::Math::Quaterniond,
		SurgSim::Math::Quaternionf> UnalignedQuaternionVariants;
TYPED_TEST_CASE(UnalignedQuaternionTests, UnalignedQuaternionVariants);



// Now we're ready to start testing...


// ==================== CONSTRUCTION & INITIALIZATION ====================

/// Test that quaternions can be constructed.
TYPED_TEST(QuaternionTests, CanConstruct)
{
	typedef typename TestFixture::Quaternion Quaternion;
	typedef typename TestFixture::Scalar     T;

	Quaternion defaultConstructed;
	Quaternion fourArg(static_cast<T>(1.0), static_cast<T>(2.0), static_cast<T>(3.0), static_cast<T>(4.0));
}

/// Test that the constructor properly initializes quaternions in the WXYZ order.
TYPED_TEST(QuaternionTests, ConstructorInitialization)
{
	typedef typename TestFixture::Quaternion Quaternion;
	typedef typename TestFixture::Scalar  T;

	Quaternion quaternion(static_cast<T>(1.03), static_cast<T>(1.04), static_cast<T>(1.05), static_cast<T>(1.06));
	EXPECT_NEAR(1.03, quaternion.w(), 1e-6) << "W wasn't properly initialized.";
	EXPECT_NEAR(1.04, quaternion.x(), 1e-6) << "X wasn't properly initialized.";
	EXPECT_NEAR(1.05, quaternion.y(), 1e-6) << "Y wasn't properly initialized.";
	EXPECT_NEAR(1.06, quaternion.z(), 1e-6) << "Z wasn't properly initialized.";
}

/// Test setting the quaternion from a 4D vector.
/// The order of components is XYZW (not WXYZ), which is why doing this may be confusing.
TYPED_TEST(QuaternionTests, InitializeFromVector4)
{
	typedef typename TestFixture::Quaternion Quaternion;
	typedef typename TestFixture::Scalar  T;

	Eigen::Matrix<T, 4, 1> vector4;
	vector4 << 1.1f, 1.2f, 1.3f, 1.4f;

	// Note: this initializes the quaternion from the vector in the **XYZW** order (not WXYZ!)
	Quaternion quaternion(vector4);
	EXPECT_NEAR(1.1, quaternion.x(), 1e-6) << "X wasn't properly initialized.";
	EXPECT_NEAR(1.2, quaternion.y(), 1e-6) << "Y wasn't properly initialized.";
	EXPECT_NEAR(1.3, quaternion.z(), 1e-6) << "Z wasn't properly initialized.";
	EXPECT_NEAR(1.4, quaternion.w(), 1e-6) << "W wasn't properly initialized.";
}

/// Test initializing from a float array.
/// The order of components is XYZW (not WXYZ), which is why doing this may be confusing.
TYPED_TEST(QuaternionTests, InitializeFromArray)
{
	typedef typename TestFixture::Quaternion Quaternion;
	typedef typename TestFixture::Scalar T;

	Quaternion quaternion(T(3.4), T(0.1), T(1.2), T(2.3));

	EXPECT_NEAR(0.1, quaternion.x(), 1e-6) << "X wasn't properly initialized.";
	EXPECT_NEAR(1.2, quaternion.y(), 1e-6) << "Y wasn't properly initialized.";
	EXPECT_NEAR(2.3, quaternion.z(), 1e-6) << "Z wasn't properly initialized.";
	EXPECT_NEAR(3.4, quaternion.w(), 1e-6) << "W wasn't properly initialized.";
}

/// Test getting an identity value usable in expressions.
TYPED_TEST(QuaternionTests, IdentityValue)
{
	typedef typename TestFixture::Quaternion Quaternion;

	Quaternion quaternion = Quaternion::Identity();
	EXPECT_NEAR(1.0, quaternion.w(), 1e-6) << "W wasn't properly initialized.";
	EXPECT_NEAR(0.0, quaternion.x(), 1e-6) << "X wasn't properly initialized.";
	EXPECT_NEAR(0.0, quaternion.y(), 1e-6) << "Y wasn't properly initialized.";
	EXPECT_NEAR(0.0, quaternion.z(), 1e-6) << "Z wasn't properly initialized.";
}

/// Test setting quaternions to identity.
TYPED_TEST(QuaternionTests, SetToIdentity)
{
	typedef typename TestFixture::Quaternion Quaternion;

	Quaternion quaternion;
	quaternion.setIdentity();
	EXPECT_NEAR(1.0, quaternion.w(), 1e-6) << "W wasn't properly initialized.";
	EXPECT_NEAR(0.0, quaternion.x(), 1e-6) << "X wasn't properly initialized.";
	EXPECT_NEAR(0.0, quaternion.y(), 1e-6) << "Y wasn't properly initialized.";
	EXPECT_NEAR(0.0, quaternion.z(), 1e-6) << "Z wasn't properly initialized.";
}

/// Test getting a zero value usable in expressions.
/// Note: many "4D" operations are not defined on Eigen::Quaternion, but can be performed on quaternion.coeffs().
TYPED_TEST(QuaternionTests, ZeroValue)
{
	typedef typename TestFixture::Quaternion Quaternion;
	typedef typename TestFixture::Scalar T;

	typedef Eigen::Matrix<T, 4, 1> Vector4;

	// There is no Quaternion::Zero(), but you can do this:
	Quaternion quaternion = Quaternion(Vector4::Zero());
	EXPECT_NEAR(0.0, quaternion.w(), 1e-6) << "W wasn't properly initialized.";
	EXPECT_NEAR(0.0, quaternion.x(), 1e-6) << "X wasn't properly initialized.";
	EXPECT_NEAR(0.0, quaternion.y(), 1e-6) << "Y wasn't properly initialized.";
	EXPECT_NEAR(0.0, quaternion.z(), 1e-6) << "Z wasn't properly initialized.";
}

/// Test setting quaternions to zero.
/// Note: many "4D" operations are not defined on Eigen::Quaternion, but can be performed on quaternion.coeffs().
TYPED_TEST(QuaternionTests, SetToZero)
{
	typedef typename TestFixture::Quaternion Quaternion;

	// There is no Quaternion::setZero(), but you can do this:
	Quaternion quaternion;
	quaternion.coeffs().setZero();
	EXPECT_NEAR(0.0, quaternion.w(), 1e-6) << "W wasn't properly initialized.";
	EXPECT_NEAR(0.0, quaternion.x(), 1e-6) << "X wasn't properly initialized.";
	EXPECT_NEAR(0.0, quaternion.y(), 1e-6) << "Y wasn't properly initialized.";
	EXPECT_NEAR(0.0, quaternion.z(), 1e-6) << "Z wasn't properly initialized.";
}

// Test conversion to and from yaml node
TYPED_TEST(QuaternionTests, YamlConvert)
{
	typedef typename TestFixture::Quaternion Quaternion;
	typedef typename TestFixture::Scalar T;

	Quaternion quaternion(T(0.1), T(1.2), T(2.3), T(3.4));

	YAML::Node node;

	ASSERT_NO_THROW(node = quaternion);

	EXPECT_TRUE(node.IsSequence());
	EXPECT_EQ(4u, node.size());

	ASSERT_NO_THROW({Quaternion expected = node.as<Quaternion>();});
	EXPECT_TRUE(quaternion.isApprox(node.as<Quaternion>()));

	// test decoding from AngleAxis
	AngleAxis angleAxis;
	angleAxis.angle() = T(0.1);
	angleAxis.axis()[0] = T(0.1);
	angleAxis.axis()[1] = T(1.2);
	angleAxis.axis()[2] = T(2.3);
	YAML::Node angleAxisNode = YAML::convert<AngleAxis>::encode(angleAxis);
	EXPECT_TRUE(Quaternion(angleAxis).isApprox(angleAxisNode.as<Quaternion>()));
}



// ==================== REPRESENTATION CONVERSIONS ====================

/// Test setting quaternions from an angle/axis rotation.
TYPED_TEST(QuaternionTests, FromAngleAxis)
{
	typedef typename TestFixture::Quaternion Quaternion;
	typedef typename TestFixture::Scalar T;

	typedef Eigen::Matrix<T, 3, 1> Vector3;

	T angle = 1.6f;
	Vector3 axis = Vector3(1.f, 2.f, 3.f).normalized();

	using SurgSim::Math::makeRotationQuaternion;

	Quaternion quaternion = makeRotationQuaternion(angle, axis);
	EXPECT_NEAR(std::cos(angle / 2.0f),            quaternion.w(), 1e-6) << "W wasn't properly initialized.";
	EXPECT_NEAR(axis.x() * std::sin(angle / 2.0f), quaternion.x(), 1e-6) << "X wasn't properly initialized.";
	EXPECT_NEAR(axis.y() * std::sin(angle / 2.0f), quaternion.y(), 1e-6) << "X wasn't properly initialized.";
	EXPECT_NEAR(axis.z() * std::sin(angle / 2.0f), quaternion.z(), 1e-6) << "X wasn't properly initialized.";
}

template<class T>
void testAngleAxis(const Eigen::Quaternion<T>& q, const Eigen::AngleAxis<T>& expectedAA,
				   bool expectNegatedQuatOppositeAxis = false)
{
	using SurgSim::Math::computeAngleAndAxis;
	using SurgSim::Math::computeAngle;

	Eigen::Matrix<T, 3, 1> axis, axisNeg;
	T angle, angleNeg;

	computeAngleAndAxis(q, &angle, &axis);
	EXPECT_NEAR(expectedAA.angle(),   angle,    1e-6) << "angle wasn't properly computed.";
	EXPECT_NEAR(expectedAA.axis()[0], axis.x(), 1e-6) << "X wasn't properly computed.";
	EXPECT_NEAR(expectedAA.axis()[1], axis.y(), 1e-6) << "Y wasn't properly computed.";
	EXPECT_NEAR(expectedAA.axis()[2], axis.z(), 1e-6) << "Y wasn't properly computed.";
	EXPECT_NEAR(expectedAA.angle(), computeAngle(q), 1e-6) << "angle wasn't properly computed by computeAngle().";

	Eigen::Quaternion<T> qNeg = SurgSim::Math::negate(q);
	computeAngleAndAxis(qNeg, &angleNeg, &axisNeg);
	EXPECT_NEAR(angle,    angleNeg,    1e-6) << "angle wasn't properly computed.";
	if (expectNegatedQuatOppositeAxis)
	{
		EXPECT_NEAR(-axis.x(), axisNeg.x(), 1e-6) << "X wasn't properly computed.";
		EXPECT_NEAR(-axis.y(), axisNeg.y(), 1e-6) << "Y wasn't properly computed.";
		EXPECT_NEAR(-axis.z(), axisNeg.z(), 1e-6) << "Y wasn't properly computed.";
	}
	else
	{
		EXPECT_NEAR(axis.x(), axisNeg.x(), 1e-6) << "X wasn't properly computed.";
		EXPECT_NEAR(axis.y(), axisNeg.y(), 1e-6) << "Y wasn't properly computed.";
		EXPECT_NEAR(axis.z(), axisNeg.z(), 1e-6) << "Y wasn't properly computed.";
	}
}

/// Test extracting an angle/axis rotation from a quaternion.
TYPED_TEST(QuaternionTests, ToAngleAxis)
{
	using SurgSim::Math::makeRotationQuaternion;
	using SurgSim::Math::computeAngleAndAxis;
	using SurgSim::Math::computeAngle;

	typedef typename TestFixture::AngleAxis AngleAxis;
	typedef typename TestFixture::Quaternion Quaternion;
	typedef typename TestFixture::Scalar T;

	typedef Eigen::Matrix<T, 3, 1> Vector3;

	Vector3 axis(T(1.0), T(2.0), T(3.0));
	AngleAxis expectedAA;
	T angle;
	axis.normalize();

	// Angle 0 in [0 pi]
	angle = T(0);
	// Expected result: 0 (to be in [0 pi]), same axis (same quaternion)
	{
		SCOPED_TRACE("Angle = 0");
		Quaternion quaternion = makeRotationQuaternion(angle, axis); // q=(1 0 0 0)
		expectedAA.angle() = angle;
		expectedAA.axis() = Vector3(1, 0, 0);

		testAngleAxis<T>(quaternion, expectedAA);
	}

	// Angle pi/2 in [0 pi]
	angle = T(M_PI_2);
	// Expected result: pi/2 (to be in [0 pi]), same axis (same quaternion)
	{
		SCOPED_TRACE("Angle = PI/2");
		Quaternion quaternion = makeRotationQuaternion(angle, axis);
		expectedAA.angle() = angle;
		expectedAA.axis() = axis;

		testAngleAxis<T>(quaternion, expectedAA);
	}

	// Angle pi-epsilon in [0 pi]
	// => cos(angle/2) = +epsilon'
	angle = T(2) * acos(std::numeric_limits<T>::epsilon());
	// Expected result: pi-epsilon (to be in [0 pi]), same axis (same quaternion)
	{
		SCOPED_TRACE("Angle = PI-epsilon");
		Quaternion quaternion = makeRotationQuaternion(angle, axis);
		expectedAA.angle() = angle;
		expectedAA.axis() = axis;

		testAngleAxis<T>(quaternion, expectedAA);
	}

	// Angle pi
	angle = T(M_PI);
	// Expected result: pi (to be in [0 pi]), same axis (same quaternion)
	// For negated quaternion, we expected angle=pi and opposite axis
	// Quaternion with w=0 are the only one for which q and -q will give different rotation axis
	{
		SCOPED_TRACE("Angle = PI");
		// Calling makeRotationQuaternion(M_PI, axis) will not set w to 0 (=cos(PI/2)) because of numerical error
		// So we set it manually to force the test
		Quaternion quaternion;
		quaternion.w() = T(0);
		quaternion.x() = axis[0];
		quaternion.y() = axis[1];
		quaternion.z() = axis[2];

		expectedAA.angle() = angle;
		expectedAA.axis() = axis;

		testAngleAxis<T>(quaternion, expectedAA, true);
	}

	// Angle pi+epsilon in [pi 2pi]
	// => cos(angle/2) = -epsilon'
	angle = T(2) * acos(-std::numeric_limits<T>::epsilon());
	// Expected result: -(pi+epsilon-2pi) (to be in [0 pi]), opposite axis (modulo 2pi + opposite quaternion)
	{
		SCOPED_TRACE("Angle = PI+epsilon");
		Quaternion quaternion = makeRotationQuaternion(angle, axis);
		expectedAA.angle() = T(2) * T(M_PI) - angle;
		expectedAA.axis() = -axis;

		testAngleAxis<T>(quaternion, expectedAA);
	}

	// Angle in [-2pi -pi]
	angle = T(-3.56);
	// Expected result: angle+2PI (to be in [0 pi]), same axis (modulo 2pi + same quaternion)
	{
		SCOPED_TRACE("Angle in [-2PI -PI]");
		Quaternion quaternion = makeRotationQuaternion(angle, axis);
		expectedAA.angle() = angle + (T)2.0 * T(M_PI);
		expectedAA.axis() = axis;

		testAngleAxis<T>(quaternion, expectedAA);
	}

	// Angle in [-pi 0]
	angle = T(-2.12);
	// Expected result: Opposite angle (to be in [0 pi]), opposite axis (opposite quaternion)
	{
		SCOPED_TRACE("Angle in [-PI 0]");
		Quaternion quaternion = makeRotationQuaternion(angle, axis);
		expectedAA.angle() = -angle;
		expectedAA.axis() = -axis;

		testAngleAxis<T>(quaternion, expectedAA);
	}

	// Angle in [0 pi]
	angle = T(0.34);
	// Expected result: Same angle (already in [0 pi]), same axis (same quaternion)
	{
		SCOPED_TRACE("Angle in [0 PI]");
		Quaternion quaternion = makeRotationQuaternion(angle, axis);
		expectedAA.angle() = angle;
		expectedAA.axis() = axis;

		testAngleAxis<T>(quaternion, expectedAA);
	}

	// Angle in [pi, 2pi]
	angle = T(4.73);
	// Expected result: -(angle-2PI) to be in [0, pi], opposite axis (modulo 2pi + opposite quaternion)
	{
		SCOPED_TRACE("Angle in [PI 2PI]");
		Quaternion quaternion = makeRotationQuaternion(angle, axis);
		expectedAA.angle() = -angle + T(2) * T(M_PI);
		expectedAA.axis() = -axis;

		testAngleAxis<T>(quaternion, expectedAA);
	}
}

/// Test setting a quaternion from a matrix.
TYPED_TEST(QuaternionTests, FromMatrix)
{
	typedef typename TestFixture::Quaternion Quaternion;
	typedef typename TestFixture::Scalar T;

	typedef Eigen::Matrix<T, 3, 3> Matrix33;

	T angle = 0.1f;
	T sinAngle = std::sin(angle);
	T cosAngle = std::cos(angle);

	Matrix33 matrix;
	matrix <<
		   cosAngle, -sinAngle, 0,
					 sinAngle, cosAngle, 0,
					 0, 0, 1;

	Quaternion quaternion(matrix);
	EXPECT_NEAR(std::cos(angle / 2), quaternion.w(), 1e-6) << "W wasn't properly computed.";
	EXPECT_NEAR(0,                 quaternion.x(), 1e-6) << "X wasn't properly computed.";
	EXPECT_NEAR(0,                 quaternion.y(), 1e-6) << "Y wasn't properly computed.";
	EXPECT_NEAR(std::sin(angle / 2), quaternion.z(), 1e-6) << "Z wasn't properly computed.";
}


/// Test setting a matrix from a quaternion.
TYPED_TEST(QuaternionTests, ToMatrix)
{
	typedef typename TestFixture::Quaternion Quaternion;
	typedef typename TestFixture::Scalar T;

	typedef Eigen::Matrix<T, 3, 3> Matrix33;

	T angle = 0.1f;
	Quaternion quaternion(std::cos(angle / 2), 0, 0, std::sin(angle / 2));

	Matrix33 expectedMatrix;
	T sinAngle = std::sin(angle);
	T cosAngle = std::cos(angle);
	expectedMatrix <<
				   cosAngle, -sinAngle, 0,
							 sinAngle, cosAngle, 0,
							 0, 0, 1;

	Matrix33 matrix1 = quaternion.matrix();
	EXPECT_NEAR(0, (matrix1 - expectedMatrix).norm(), 9e-6) << "The rotation matrix wasn't properly computed" <<
			" by matrix().";
	Matrix33 matrix2 = quaternion.toRotationMatrix();
	EXPECT_NEAR(0, (matrix2 - expectedMatrix).norm(), 9e-6) << "The rotation matrix wasn't properly computed" <<
			" by toRotationMatrix().";
}

// ==================== ARITHMETIC ====================

/// Test quaternion conjugate.
TYPED_TEST(QuaternionTests, Conjugate)
{
	typedef typename TestFixture::Quaternion Quaternion;

	Quaternion q(2.1f, 2.2f, 2.3f, 2.4f);
	Quaternion n = q.conjugate();
	EXPECT_NEAR(2.1,  n.w(), 1e-6) << "W wasn't properly initialized.";
	EXPECT_NEAR(-2.2, n.x(), 1e-6) << "X wasn't properly initialized.";
	EXPECT_NEAR(-2.3, n.y(), 1e-6) << "Y wasn't properly initialized.";
	EXPECT_NEAR(-2.4, n.z(), 1e-6) << "Z wasn't properly initialized.";
}

/// Test quaternion inverse.
TYPED_TEST(QuaternionTests, Inverse)
{
	typedef typename TestFixture::Quaternion Quaternion;
	typedef typename TestFixture::Scalar T;

	Quaternion q(2.1f, 2.2f, 2.3f, 2.4f);
	Quaternion n = q.inverse();
	T scale = q.squaredNorm();

	EXPECT_NEAR(2.1 / scale,  n.w(), 1e-6) << "W wasn't properly initialized.";
	EXPECT_NEAR(-2.2 / scale, n.x(), 1e-6) << "X wasn't properly initialized.";
	EXPECT_NEAR(-2.3 / scale, n.y(), 1e-6) << "Y wasn't properly initialized.";
	EXPECT_NEAR(-2.4 / scale, n.z(), 1e-6) << "Z wasn't properly initialized.";

	Quaternion qn = q * n;
	EXPECT_NEAR(1.0, qn.w(), 1e-6) << "W of q * q^1 is messed up.";
	EXPECT_NEAR(0.0, qn.x(), 1e-6) << "X of q * q^1 is messed up.";
	EXPECT_NEAR(0.0, qn.y(), 1e-6) << "Y of q * q^1 is messed up.";
	EXPECT_NEAR(0.0, qn.z(), 1e-6) << "Z of q * q^1 is messed up.";

	Quaternion nq = n * q;
	EXPECT_NEAR(1.0, nq.w(), 1e-6) << "W of q^1 * q is messed up.";
	EXPECT_NEAR(0.0, nq.x(), 1e-6) << "X of q^1 * q is messed up.";
	EXPECT_NEAR(0.0, nq.y(), 1e-6) << "Y of q^1 * q is messed up.";
	EXPECT_NEAR(0.0, nq.z(), 1e-6) << "Z of q^1 * q is messed up.";
}

/// Test quaternion rotation of vectors.
TYPED_TEST(QuaternionTests, ApplyToVector)
{
	typedef typename TestFixture::Quaternion Quaternion;
	typedef typename TestFixture::Scalar T;

	typedef Eigen::Matrix<T, 3, 1> Vector;

	using SurgSim::Math::makeRotationQuaternion;
	Quaternion q = makeRotationQuaternion(static_cast<T>(M_PI_2), Vector(1, 0, 0));

	// You can use this, which is actually more efficient if you can manage to apply the same matrix more than once:
	Vector qx = q.matrix() * Vector::UnitX();
	EXPECT_TRUE(qx.isApprox(Vector::UnitX(), 1e-6f)) << qx;
	// ...or this, which clearly isn't very recommended by the API designers:
	Vector qx2 = q._transformVector(Vector::UnitX());
	EXPECT_TRUE(qx2.isApprox(Vector::UnitX(), 1e-6f)) << qx2;

	Vector qy = q.matrix() * Vector::UnitY();
	EXPECT_TRUE(qy.isApprox(Vector::UnitZ(), 1e-6f)) << qy;

	Vector qz = q.matrix() * Vector::UnitZ();
	EXPECT_TRUE(qz.isApprox(-Vector::UnitY(), 1e-6f)) << qz;
}

/// Quaternion norm and its square.
TYPED_TEST(QuaternionTests, NormAndSquared)
{
	typedef typename TestFixture::Quaternion Quaternion;

	Quaternion q(3.1f, 3.4f, 3.7f, 4.0f);
	// sum of the squares of the components
	double expectedSumSquares = 50.86;

	EXPECT_NEAR(expectedSumSquares, q.squaredNorm(), 1e-6);
	EXPECT_NEAR(sqrt(expectedSumSquares), q.norm(), 1e-6);
}

/// Normalization of quaternions.
TYPED_TEST(QuaternionTests, Normalize)
{
	typedef typename TestFixture::Quaternion Quaternion;

	Quaternion q(3.1f, 3.4f, 3.7f, 4.0f);
	// sum of the squares of the components
	double expectedSumSquares = 50.86;

	EXPECT_NEAR(sqrt(expectedSumSquares), q.norm(), 1e-6);

	// normalized() RETURNS the normalized quaternion, leaving original unchanged.
	Quaternion qNorm = q.normalized();
	EXPECT_NEAR(1, qNorm.norm(), 1e-6);
	EXPECT_NEAR(sqrt(expectedSumSquares), q.norm(), 1e-6);

	// normalize() NORMALIZES the quaternion, modifying it.
	q.normalize();
	EXPECT_NEAR(1, q.norm(), 1e-6);
	EXPECT_NEAR(0, (qNorm.coeffs() - q.coeffs()).norm(), 1e-6);
}

// ==================== TYPE CONVERSION ====================

/// Typecasting quaternions (double <-> float conversions).
TYPED_TEST(QuaternionTests, TypeCasting)
{
	typedef typename TestFixture::Quaternion Quaternion;
	typedef Eigen::Quaternion<double> Quaterniond;
	typedef Eigen::Quaternion<float>  Quaternionf;

	Quaternion q(12.1f, 12.2f, 12.3f, 12.4f);
	// Ugh, "template" is required to get this to parse properly.  This is
	// triggered because the test is a part of a template class; you don't
	// need to do this in a non-template context.
	Quaterniond qd = q.template cast<double>();
	EXPECT_NEAR(q.norm(), qd.norm(), 1e-6);
	Quaternionf qf = q.template cast<float>();
	EXPECT_NEAR(q.norm(), qf.norm(), 1e-4);
}

// ==================== MISCELLANEOUS ====================

/// Reading from and writing to arrays or blocks of double/float in memory.
TYPED_TEST(QuaternionTests, ArrayReadWrite)
{
	typedef typename TestFixture::Quaternion Quaternion;
	typedef typename TestFixture::Scalar T;

	const T inputArray[5]  = { 12.1f, 12.2f, 12.3f, 12.4f, 12.5f };
	T outputArray[5];

	// Note that with the current versions of Eigen, you CANNOT say this:
	//   Eigen::Map<SurgSim::Math::Quaternion> q(array);
	// because you can't map Eigen::Quaternion with non-zero flags!
	// But mapping a default (AutoAlign) quaternion should be safe even if the buffer is not aligned.

	Eigen::Map<const Eigen::Quaternion<T> > q_in(inputArray);
	Quaternion q1 = q_in;

	Eigen::Map<Eigen::Quaternion<T> > q_out(outputArray);
	q_out = q1;

	for (int i = 0;  i < 4;  ++i)
	{
		EXPECT_NEAR(inputArray[i], outputArray[i], 1e-6);
	}
}

/// Test quaternion negation.
TYPED_TEST(QuaternionTests, Negate)
{
	using SurgSim::Math::negate;

	typedef typename TestFixture::Scalar T;
	typedef Eigen::Quaternion<T> Quaternion;

	// Test that 2 quaternions are opposite if they are not equal but give the same rotation
	for (size_t numLoop = 0; numLoop < 100; numLoop++)
	{
		Quaternion q(Eigen::Matrix<T, 4, 1>::Random());
		q.normalize();
		Quaternion qNeg = negate(q);
		EXPECT_FALSE(q.isApprox(qNeg));

		typename Quaternion::Matrix3 m = q.toRotationMatrix();
		typename Quaternion::Matrix3 mNeg = qNeg.toRotationMatrix();
		EXPECT_TRUE(m.isApprox(mNeg));
	}
}

// ==================== SLERP ====================

/// Test quaternion interpolation.
TYPED_TEST(QuaternionTests, SlerpInterpolation)
{
	using SurgSim::Math::negate;

	typedef typename TestFixture::Scalar T;
	typedef Eigen::Quaternion<T> Quaternion;

	for (size_t numLoop = 0; numLoop < 100; numLoop++)
	{
		Quaternion q;
		Quaternion q0(Eigen::Matrix<T, 4, 1>::Random());
		Quaternion q1(Eigen::Matrix<T, 4, 1>::Random());
		q0.normalize();
		q1.normalize();

		q = SurgSim::Math::interpolate(q0, q1, static_cast<T>(0.0));
		EXPECT_TRUE(q.isApprox(q0) || q.isApprox(negate(q0)));
		q = SurgSim::Math::interpolate(q0, q1, static_cast<T>(1.0));
		EXPECT_TRUE(q.isApprox(q1) || q.isApprox(negate(q1)));

		q = SurgSim::Math::interpolate(q0, q1, static_cast<T>(0.234));
		EXPECT_FALSE(q.isApprox(q0) || q.isApprox(negate(q0)));
		EXPECT_FALSE(q.isApprox(q1) || q.isApprox(negate(q1)));

		q = SurgSim::Math::interpolate(q0, q1, static_cast<T>(0.5));
		EXPECT_FALSE(q.isApprox(q0) || q.isApprox(negate(q0)));
		EXPECT_FALSE(q.isApprox(q1) || q.isApprox(negate(q1)));
		// At t=0.5, the interpolation should return (q0 + q1)/2 normalized
		// c.f. http://en.wikipedia.org/wiki/Slerp
		// If the quaternions are over PI angle, the slerp will interpolate between q0 and -q1
		// in this case, the interpolation is (q0 - q1)/2 normalized
		// From our specification, both quaternions could be considered negative, so we extend
		// the tests to these possibilities as well:
		// (-q0 + q1) / 2 normalized
		// (-q0 - q1) / 2 normalized
		Quaternion qHalf0((q0.coeffs() + q1.coeffs()) * 0.5);
		Quaternion qHalf1((q0.coeffs() - q1.coeffs()) * 0.5);
		Quaternion qHalf2((-q0.coeffs() + q1.coeffs()) * 0.5);
		Quaternion qHalf3((-q0.coeffs() - q1.coeffs()) * 0.5);
		qHalf0.normalize();
		qHalf1.normalize();
		qHalf2.normalize();
		qHalf3.normalize();
		EXPECT_TRUE(q.isApprox(qHalf0) || q.isApprox(qHalf1) ||
					q.isApprox(qHalf2) || q.isApprox(qHalf3));

		q = SurgSim::Math::interpolate(q0, q1, static_cast<T>(0.874));
		EXPECT_FALSE(q.isApprox(q0) || q.isApprox(negate(q0)));
		EXPECT_FALSE(q.isApprox(q1) || q.isApprox(negate(q1)));
	}
}

// TO DO:
// testing numerical validity
// testing for denormalized numbers
// testing degeneracy (norm near 0)
// compute an orthonormal frame based on a given normal (z-axis)
// Euler angles (various conventions)
// power/slerp
