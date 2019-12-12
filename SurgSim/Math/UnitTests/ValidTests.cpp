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

/// \file
/// Tests that exercise the isValid() functions.

#include <limits>
#include <iostream>
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Valid.h"
#include <Eigen/Core>
#include "gtest/gtest.h"

// Define test fixture class templates.
// We don't really need fixtures as such, but the templatization encodes type.

template <class T>
class ValidTests : public testing::Test
{
public:
	typedef T Scalar;
};

// This used to contain aligned (via Eigen::AutoAlign) matrix type aliases, but we got rid of those.
typedef ::testing::Types<double, float> FloatingPointVariants;
TYPED_TEST_CASE(ValidTests, FloatingPointVariants);

// Now we're ready to start testing...


TYPED_TEST(ValidTests, ValidScalars)
{
	typedef typename TestFixture::Scalar Scalar;
	using SurgSim::Math::isValid;

	EXPECT_TRUE(isValid(static_cast<Scalar>(0)));
	EXPECT_TRUE(isValid(static_cast<Scalar>(1)));
	EXPECT_TRUE(isValid(std::numeric_limits<Scalar>::denorm_min()));
	EXPECT_FALSE(isValid(std::numeric_limits<Scalar>::quiet_NaN()));
	EXPECT_FALSE(isValid(std::numeric_limits<Scalar>::signaling_NaN()));
	EXPECT_FALSE(isValid(std::numeric_limits<Scalar>::infinity()));
}

TYPED_TEST(ValidTests, SubnormalScalars)
{
	typedef typename TestFixture::Scalar Scalar;
	using SurgSim::Math::isSubnormal;

	EXPECT_FALSE(isSubnormal(static_cast<Scalar>(0)));
	EXPECT_FALSE(isSubnormal(static_cast<Scalar>(1)));
	EXPECT_TRUE(isSubnormal(std::numeric_limits<Scalar>::denorm_min()));
	EXPECT_FALSE(isSubnormal(std::numeric_limits<Scalar>::quiet_NaN()));
	EXPECT_FALSE(isSubnormal(std::numeric_limits<Scalar>::signaling_NaN()));
	EXPECT_FALSE(isSubnormal(std::numeric_limits<Scalar>::infinity()));
}

TYPED_TEST(ValidTests, SubnormalArithmetic)
{
	typedef typename TestFixture::Scalar Scalar;
	using SurgSim::Math::isSubnormal;

	Scalar x = 1;
	EXPECT_FALSE(isSubnormal(x));

	int normalSteps;
	for (normalSteps = 0;  normalSteps < 1000000;  ++normalSteps)
	{
		if (isSubnormal(x) || (x == 0))
		{
			break;
		}
		x /= 2;
	}
	EXPECT_GT(normalSteps, 0);

	int subnormalSteps;
	for (subnormalSteps = 0;  subnormalSteps < 1000000;  ++subnormalSteps)
	{
		if (!isSubnormal(x) || (x == 0))
		{
			break;
		}
		x /= 2;
	}
	EXPECT_GT(subnormalSteps, 0);
}

template <typename T>
static void matrixCheckHelper(const T& validMatrix)
{
	// Assumes T is a matrix, 2x2 or larger

	typedef T Matrix;
	typedef typename Matrix::Scalar Scalar;

	using SurgSim::Math::isValid;
	using SurgSim::Math::isSubnormal;

	{
		Matrix matrix = validMatrix;
		EXPECT_TRUE(isValid(matrix));
		EXPECT_FALSE(isSubnormal(matrix));
		matrix(0, 1) = std::numeric_limits<Scalar>::denorm_min();
		EXPECT_TRUE(isValid(matrix));
		EXPECT_TRUE(isSubnormal(matrix));
	}
	{
		Matrix matrix = validMatrix;
		EXPECT_TRUE(isValid(matrix));
		EXPECT_FALSE(isSubnormal(matrix));
		matrix(0, 0) = std::numeric_limits<Scalar>::infinity();
		EXPECT_FALSE(isValid(matrix));
		EXPECT_FALSE(isSubnormal(matrix));
		matrix(1, 1) = std::numeric_limits<Scalar>::denorm_min();
		EXPECT_FALSE(isValid(matrix));
		EXPECT_TRUE(isSubnormal(matrix));
	}
	{
		Matrix matrix = validMatrix;
		EXPECT_TRUE(isValid(matrix));
		EXPECT_FALSE(isSubnormal(matrix));
		matrix(1, 0) = std::numeric_limits<Scalar>::quiet_NaN();
		EXPECT_FALSE(isValid(matrix));
		EXPECT_FALSE(isSubnormal(matrix));
	}
}

TYPED_TEST(ValidTests, MatrixChecks)
{
	typedef typename TestFixture::Scalar Scalar;

	{
		Eigen::Matrix<Scalar, 2, 2, Eigen::RowMajor> matrix;
		matrix.setIdentity();
		matrixCheckHelper(matrix);
		matrix.setZero();
		matrixCheckHelper(matrix);
	}
	{
		Eigen::Matrix<Scalar, 3, 3, Eigen::ColMajor> matrix;
		matrix.setIdentity();
		matrixCheckHelper(matrix);
	}
	{
		Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor> matrix;
		matrix.setIdentity();
		matrixCheckHelper(matrix);
	}
	{
		Eigen::Matrix<Scalar, 4, 4, Eigen::ColMajor> matrix;
		matrix.setIdentity();
		matrixCheckHelper(matrix);
	}

	{
		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> matrix;
		matrix.setIdentity(11, 11);
		matrixCheckHelper(matrix);
	}
}

template <typename T>
static void vectorCheckHelper(const T& validVector)
{
	// Assumes T is a vector, size 2 or larger

	typedef T Vector;
	typedef typename Vector::Scalar Scalar;

	using SurgSim::Math::isValid;
	using SurgSim::Math::isSubnormal;

	{
		Vector vector = validVector;
		EXPECT_TRUE(isValid(vector));
		EXPECT_FALSE(isSubnormal(vector));
		vector[0] = static_cast<Scalar>(1);
		EXPECT_TRUE(isValid(vector));
		EXPECT_FALSE(isSubnormal(vector));
	}
	{
		Vector vector = validVector;
		EXPECT_TRUE(isValid(vector));
		EXPECT_FALSE(isSubnormal(vector));
		vector[1] = std::numeric_limits<Scalar>::denorm_min();
		EXPECT_TRUE(isValid(vector));
		EXPECT_TRUE(isSubnormal(vector));
	}
	{
		Vector vector = validVector;
		EXPECT_TRUE(isValid(vector));
		EXPECT_FALSE(isSubnormal(vector));
		vector[0] = std::numeric_limits<Scalar>::infinity();
		EXPECT_FALSE(isValid(vector));
		EXPECT_FALSE(isSubnormal(vector));
		vector[1] = std::numeric_limits<Scalar>::denorm_min();
		EXPECT_FALSE(isValid(vector));
		EXPECT_TRUE(isSubnormal(vector));
	}
	{
		Vector vector = validVector;
		EXPECT_TRUE(isValid(vector));
		EXPECT_FALSE(isSubnormal(vector));
		vector[1] = std::numeric_limits<Scalar>::quiet_NaN();
		EXPECT_FALSE(isValid(vector));
		EXPECT_FALSE(isSubnormal(vector));
	}
}

TYPED_TEST(ValidTests, VectorChecks)
{
	typedef typename TestFixture::Scalar Scalar;

	{
		Eigen::Matrix<Scalar, 2, 1> vector;
		vector.setZero();
		vectorCheckHelper(vector);
		vector.setZero();
		vectorCheckHelper(vector);
	}
	{
		Eigen::Matrix<Scalar, 3, 1> vector;
		vector.setZero();
		vectorCheckHelper(vector);
	}
	{
		Eigen::Matrix<Scalar, 4, 1> vector;
		vector.setZero();
		vectorCheckHelper(vector);
	}
	{
		Eigen::Matrix<Scalar, 4, 1, Eigen::AutoAlign> vector;
		vector.setZero();
		vectorCheckHelper(vector);
	}

	{
		Eigen::Matrix<Scalar, Eigen::Dynamic, 1> vector;
		vector.setZero(11);
		vectorCheckHelper(vector);
	}
}

TYPED_TEST(ValidTests, QuaternionChecks)
{
	typedef typename TestFixture::Scalar Scalar;
	using SurgSim::Math::isValid;
	using SurgSim::Math::isSubnormal;

	Eigen::Quaternion<Scalar> quaternion(1, 0, 0, 0);
	EXPECT_TRUE(isValid(quaternion));
	EXPECT_FALSE(isSubnormal(quaternion));

	quaternion.x() = std::numeric_limits<Scalar>::denorm_min();
	EXPECT_TRUE(isValid(quaternion));
	EXPECT_TRUE(isSubnormal(quaternion));

	quaternion = Eigen::Quaternion<Scalar>(std::numeric_limits<Scalar>::infinity(), 0, 0, 0);
	EXPECT_FALSE(isValid(quaternion));
	EXPECT_FALSE(isSubnormal(quaternion));

	quaternion.z() = std::numeric_limits<Scalar>::denorm_min();
	EXPECT_FALSE(isValid(quaternion));
	EXPECT_TRUE(isSubnormal(quaternion));
}

TYPED_TEST(ValidTests, AngleAxisChecks)
{
	typedef typename TestFixture::Scalar Scalar;
	using SurgSim::Math::isValid;
	using SurgSim::Math::isSubnormal;

	Eigen::AngleAxis<Scalar> rotation = Eigen::AngleAxis<Scalar>::Identity();
	EXPECT_TRUE(isValid(rotation));
	EXPECT_FALSE(isSubnormal(rotation));

	rotation.angle() = std::numeric_limits<Scalar>::denorm_min();
	EXPECT_TRUE(isValid(rotation));
	EXPECT_TRUE(isSubnormal(rotation));

	rotation = Eigen::AngleAxis<Scalar>::Identity();
	rotation.axis()[2] = std::numeric_limits<Scalar>::denorm_min();
	EXPECT_TRUE(isValid(rotation));
	EXPECT_TRUE(isSubnormal(rotation));

	rotation = Eigen::AngleAxis<Scalar>::Identity();
	rotation.angle() = std::numeric_limits<Scalar>::infinity();
	EXPECT_FALSE(isValid(rotation));
	EXPECT_FALSE(isSubnormal(rotation));
	rotation.axis()[1] = std::numeric_limits<Scalar>::denorm_min();
	EXPECT_FALSE(isValid(rotation));
	EXPECT_TRUE(isSubnormal(rotation));

	rotation = Eigen::AngleAxis<Scalar>::Identity();
	rotation.axis()[0] = std::numeric_limits<Scalar>::quiet_NaN();
	EXPECT_FALSE(isValid(rotation));
	EXPECT_FALSE(isSubnormal(rotation));
	rotation.angle() = std::numeric_limits<Scalar>::denorm_min();
	EXPECT_FALSE(isValid(rotation));
	EXPECT_TRUE(isSubnormal(rotation));
}

TYPED_TEST(ValidTests, Rotation2DChecks)
{
	typedef typename TestFixture::Scalar Scalar;
	using SurgSim::Math::isValid;
	using SurgSim::Math::isSubnormal;

	Eigen::Rotation2D<Scalar> rotation(0);
	EXPECT_TRUE(isValid(rotation));
	EXPECT_FALSE(isSubnormal(rotation));

	rotation.angle() = static_cast<Scalar>(1);
	EXPECT_TRUE(isValid(rotation));
	EXPECT_FALSE(isSubnormal(rotation));

	rotation.angle() = std::numeric_limits<Scalar>::denorm_min();
	EXPECT_TRUE(isValid(rotation));
	EXPECT_TRUE(isSubnormal(rotation));

	rotation = Eigen::Rotation2D<Scalar>(std::numeric_limits<Scalar>::infinity());
	EXPECT_FALSE(isValid(rotation));
	EXPECT_FALSE(isSubnormal(rotation));
}

template <typename T>
static void transformCheckHelper()
{
	// Assumes T is an Eigen::Transform type of some sort

	typedef T Transform;
	typedef typename Transform::Scalar Scalar;

	using SurgSim::Math::isValid;
	using SurgSim::Math::isSubnormal;

	Transform transform = Transform::Identity();
	EXPECT_TRUE(isValid(transform));
	EXPECT_FALSE(isSubnormal(transform));

	transform(1, 1) = std::numeric_limits<Scalar>::denorm_min();
	EXPECT_TRUE(isValid(transform));
	EXPECT_TRUE(isSubnormal(transform));

	transform = Transform::Identity();
	transform(0, 0) = std::numeric_limits<Scalar>::quiet_NaN();
	EXPECT_FALSE(isValid(transform));
	EXPECT_FALSE(isSubnormal(transform));

	transform(0, 1) = std::numeric_limits<Scalar>::denorm_min();
	EXPECT_FALSE(isValid(transform));
	EXPECT_TRUE(isSubnormal(transform));
}

TYPED_TEST(ValidTests, TransformChecks)
{
	typedef typename TestFixture::Scalar Scalar;
	using SurgSim::Math::isValid;
	using SurgSim::Math::isSubnormal;

	transformCheckHelper<Eigen::Transform<Scalar, 2, Eigen::Isometry>>();
	transformCheckHelper<Eigen::Transform<Scalar, 3, Eigen::Isometry>>();
	transformCheckHelper<Eigen::Transform<Scalar, 4, Eigen::Isometry>>();
	transformCheckHelper<Eigen::Transform<Scalar, 4, Eigen::Isometry>>();
	transformCheckHelper<Eigen::Transform<Scalar, 4, Eigen::Affine>>();
	transformCheckHelper<Eigen::Transform<Scalar, 4, Eigen::AffineCompact>>();
}

TYPED_TEST(ValidTests, ClearSubnormalScalars)
{
	typedef typename TestFixture::Scalar Scalar;
	using SurgSim::Math::setSubnormalToZero;
	using SurgSim::Math::isValid;
	Scalar x;

	x = 0;
	EXPECT_FALSE(setSubnormalToZero(&x));
	EXPECT_EQ(0, x);

	x = -1;
	EXPECT_FALSE(setSubnormalToZero(&x));
	EXPECT_EQ(-1, x);

	x = std::numeric_limits<Scalar>::infinity();
	EXPECT_FALSE(setSubnormalToZero(&x));
	EXPECT_FALSE(isValid(x));

	x = std::numeric_limits<Scalar>::quiet_NaN();
	EXPECT_FALSE(setSubnormalToZero(&x));
	EXPECT_FALSE(isValid(x));

	x = std::numeric_limits<Scalar>::signaling_NaN();
	EXPECT_FALSE(setSubnormalToZero(&x));
	EXPECT_FALSE(isValid(x));

	x = std::numeric_limits<Scalar>::denorm_min();
	EXPECT_TRUE(setSubnormalToZero(&x));
	EXPECT_EQ(0, x);
}

template <typename T>
static void compareMatrices(const T& a, const T& b)
{
	typedef T Matrix;
	using Eigen::Index;

	EXPECT_EQ(a.rows(), b.rows());
	EXPECT_EQ(a.cols(), b.cols());

	const Index numColumns = std::min(a.cols(), b.cols());
	const Index numRows = std::min(a.rows(), b.rows());

	for (Index j = 0; j < numColumns; ++j)
	{
		for (Index i = 0; i < numRows; ++i)
		{
			bool isValidAij = SurgSim::Math::isValid(a.coeff(i, j));
			bool isValidBij = SurgSim::Math::isValid(b.coeff(i, j));
			EXPECT_EQ(isValidAij, isValidBij) << "i = " << i << ", j = " << j <<
				", Aij = " << a.coeff(i, j) << ", Bij = " << b.coeff(i, j);
			if (isValidAij && isValidBij)
			{
				// In general, floating point equality checks are bad, but here they are needed.
				EXPECT_EQ(a.coeff(i, j), b.coeff(i, j)) << "i = " << i << ", j = " << j;
			}
		}
	}
}

template <typename T>
static void matrixSetSubnormalHelper(const T& validMatrix)
{
	// Assumes T is a matrix, 2x2 or larger

	typedef T Matrix;
	typedef typename Matrix::Scalar Scalar;

	EXPECT_TRUE(SurgSim::Math::isValid(validMatrix));
	EXPECT_FALSE(SurgSim::Math::isSubnormal(validMatrix));

	using SurgSim::Math::setSubnormalToZero;

	{
		Matrix a = validMatrix;
		Matrix b = a;
		EXPECT_FALSE(setSubnormalToZero(&a));
		EXPECT_FALSE(setSubnormalToZero(&b));
		compareMatrices(a, b);
	}
	{
		Matrix a = validMatrix;
		a(0, 1) = 0;
		Matrix b = a;
		b(0, 1) = std::numeric_limits<Scalar>::denorm_min();
		EXPECT_FALSE(setSubnormalToZero(&a));
		EXPECT_TRUE(setSubnormalToZero(&b));
		compareMatrices(a, b);
	}
	{
		Matrix a = validMatrix;
		a(0, 0) = std::numeric_limits<Scalar>::infinity();
		Matrix b = a;
		EXPECT_FALSE(setSubnormalToZero(&a));
		EXPECT_FALSE(setSubnormalToZero(&b));
		compareMatrices(a, b);
	}
	{
		Matrix a = validMatrix;
		a(1, 0) = std::numeric_limits<Scalar>::quiet_NaN();
		a(1, 1) = 0;
		Matrix b = a;
		b(1, 1) = std::numeric_limits<Scalar>::denorm_min();
		EXPECT_FALSE(setSubnormalToZero(&a));
		EXPECT_TRUE(setSubnormalToZero(&b));
		compareMatrices(a, b);
	}
}

TYPED_TEST(ValidTests, ClearSubnormalMatrix)
{
	typedef typename TestFixture::Scalar Scalar;

	{
		Eigen::Matrix<Scalar, 2, 2, Eigen::RowMajor> matrix;
		matrix.setConstant(123);
		matrixSetSubnormalHelper(matrix);
		matrix.setZero();
		matrixSetSubnormalHelper(matrix);
	}
	{
		Eigen::Matrix<Scalar, 3, 3, Eigen::ColMajor> matrix;
		matrix.setConstant(123);
		matrixSetSubnormalHelper(matrix);
	}
	{
		Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor> matrix;
		matrix.setConstant(123);
		matrixSetSubnormalHelper(matrix);
	}
	{
		Eigen::Matrix<Scalar, 4, 4, Eigen::ColMajor> matrix;
		matrix.setConstant(123);
		matrixSetSubnormalHelper(matrix);
	}

	{
		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> matrix;
		matrix.setConstant(11, 13, static_cast<Scalar>(123));
		matrixSetSubnormalHelper(matrix);
	}
}

template <typename T>
static void vectorSetSubnormalHelper(const T& validVector)
{
	// Assumes T is a vector, size 2 or larger

	typedef T Vector;
	typedef typename Vector::Scalar Scalar;

	using SurgSim::Math::setSubnormalToZero;

	{
		Vector a = validVector;
		Vector b = a;
		EXPECT_FALSE(setSubnormalToZero(&a));
		EXPECT_FALSE(setSubnormalToZero(&b));
		compareMatrices(a, b);
	}
	{
		Vector a = validVector;
		a[0] = 0;
		Vector b = a;
		b[0] = std::numeric_limits<Scalar>::denorm_min();
		EXPECT_FALSE(setSubnormalToZero(&a));
		EXPECT_TRUE(setSubnormalToZero(&b));
		compareMatrices(a, b);
	}
	{
		Vector a = validVector;
		a[1] = std::numeric_limits<Scalar>::infinity();
		Vector b = a;
		EXPECT_FALSE(setSubnormalToZero(&a));
		EXPECT_FALSE(setSubnormalToZero(&b));
		compareMatrices(a, b);
	}
	{
		Vector a = validVector;
		a[0] = std::numeric_limits<Scalar>::quiet_NaN();
		a[1] = 0;
		Vector b = a;
		b[1] = std::numeric_limits<Scalar>::denorm_min();
		EXPECT_FALSE(setSubnormalToZero(&a));
		EXPECT_TRUE(setSubnormalToZero(&b));
		compareMatrices(a, b);
	}
}

TYPED_TEST(ValidTests, ClearSubnormalVector)
{
	typedef typename TestFixture::Scalar Scalar;

	{
		Eigen::Matrix<Scalar, 2, 1> vector;
		vector.setConstant(543);
		vectorSetSubnormalHelper(vector);
		vector.setZero();
		vectorSetSubnormalHelper(vector);
	}
	{
		Eigen::Matrix<Scalar, 3, 1> vector;
		vector.setConstant(543);
		vectorSetSubnormalHelper(vector);
	}
	{
		Eigen::Matrix<Scalar, 4, 1> vector;
		vector.setConstant(543);
		vectorSetSubnormalHelper(vector);
	}
	{
		Eigen::Matrix<Scalar, 4, 1, Eigen::AutoAlign> vector;
		vector.setConstant(543);
		vectorSetSubnormalHelper(vector);
	}

	{
		Eigen::Matrix<Scalar, Eigen::Dynamic, 1> vector;
		vector.setConstant(21, static_cast<Scalar>(543));
		vectorSetSubnormalHelper(vector);
	}
}

TYPED_TEST(ValidTests, ClearSubnormalQuaternion)
{
	typedef typename TestFixture::Scalar Scalar;
	using SurgSim::Math::setSubnormalToZero;

	Eigen::Quaternion<Scalar> quaternion(1, 0, 0, 0);
	EXPECT_FALSE(setSubnormalToZero(&quaternion));

	{
		Eigen::Quaternion<Scalar> q2 = quaternion;
		q2.x() = std::numeric_limits<Scalar>::denorm_min();
		EXPECT_TRUE(setSubnormalToZero(&q2));
		compareMatrices(quaternion.coeffs(), q2.coeffs());
	}
	{
		Eigen::Quaternion<Scalar> q3 = quaternion;
		q3.y() = std::numeric_limits<Scalar>::infinity();
		EXPECT_FALSE(setSubnormalToZero(&q3));
		EXPECT_FALSE(SurgSim::Math::isValid(q3));

		Eigen::Quaternion<Scalar> q4 = q3;
		q4.z() = std::numeric_limits<Scalar>::denorm_min();
		EXPECT_TRUE(setSubnormalToZero(&q4));
		compareMatrices(q3.coeffs(), q4.coeffs());
	}
}

template <typename T>
static void compareAngleAxis(const T& a, const T& b)
{
	bool isValidAngleA = SurgSim::Math::isValid(a.angle());
	bool isValidAngleB = SurgSim::Math::isValid(b.angle());

	EXPECT_EQ(isValidAngleA, isValidAngleB) << " angle A = " << a.angle() << ", B = " << b.angle();
	if (isValidAngleA && isValidAngleB)
	{
		// In general, floating point equality checks are bad, but here they are needed.
		EXPECT_EQ(a.angle(), b.angle());
	}

	compareMatrices(a.axis(), b.axis());
}

TYPED_TEST(ValidTests, ClearSubnormalAngleAxis)
{
	typedef typename TestFixture::Scalar Scalar;
	using SurgSim::Math::setSubnormalToZero;

	typedef Eigen::AngleAxis<Scalar> AngleAxis;
	typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

	AngleAxis rotation;

	rotation = AngleAxis(-1, Vector3(1, 2, 3));
	EXPECT_FALSE(setSubnormalToZero(&rotation));
	compareAngleAxis(AngleAxis(-1, Vector3(1, 2, 3)), rotation);

	rotation = AngleAxis(std::numeric_limits<Scalar>::denorm_min(), Vector3(1, 2, 3));
	EXPECT_TRUE(setSubnormalToZero(&rotation));
	compareAngleAxis(AngleAxis(0, Vector3(1, 2, 3)), rotation);

	rotation = AngleAxis(-1, Vector3(std::numeric_limits<Scalar>::denorm_min(), 2, 3));
	EXPECT_TRUE(setSubnormalToZero(&rotation));
	compareAngleAxis(AngleAxis(-1, Vector3(0, 2, 3)), rotation);

	rotation = AngleAxis(-1, Vector3(1, std::numeric_limits<Scalar>::infinity(), 3));
	EXPECT_FALSE(setSubnormalToZero(&rotation));
	compareAngleAxis(AngleAxis(-1, Vector3(1, std::numeric_limits<Scalar>::infinity(), 3)), rotation);

	rotation = AngleAxis(std::numeric_limits<Scalar>::denorm_min(),
						 Vector3(1, 2, std::numeric_limits<Scalar>::infinity()));
	EXPECT_TRUE(setSubnormalToZero(&rotation));
	compareAngleAxis(AngleAxis(0, Vector3(1, 2, std::numeric_limits<Scalar>::infinity())), rotation);
}

TYPED_TEST(ValidTests, ClearSubnormalRotation2D)
{
	typedef typename TestFixture::Scalar Scalar;
	using SurgSim::Math::setSubnormalToZero;

	Eigen::Rotation2D<Scalar> rotation(0);
	EXPECT_FALSE(setSubnormalToZero(&rotation));
	EXPECT_EQ(0, rotation.angle());

	rotation.angle() = static_cast<Scalar>(1);
	EXPECT_FALSE(setSubnormalToZero(&rotation));
	EXPECT_EQ(1, rotation.angle());

	rotation.angle() = std::numeric_limits<Scalar>::denorm_min();
	EXPECT_TRUE(setSubnormalToZero(&rotation));
	EXPECT_EQ(0, rotation.angle());

	rotation = Eigen::Rotation2D<Scalar>(std::numeric_limits<Scalar>::infinity());
	EXPECT_FALSE(setSubnormalToZero(&rotation));
	EXPECT_FALSE(SurgSim::Math::isValid(rotation.angle()));
}

template <typename T>
static void transformSetSubnormalHelper()
{
	// Assumes T is an Eigen::Transform type of some sort

	typedef T Transform;
	typedef typename Transform::Scalar Scalar;

	using SurgSim::Math::setSubnormalToZero;

	Transform transform = Transform::Identity();
	EXPECT_FALSE(setSubnormalToZero(&transform));
	compareMatrices(Transform::Identity().matrix(), transform.matrix());

	{
		Transform t2 = transform;
		t2(0, 1) = std::numeric_limits<Scalar>::denorm_min();
		EXPECT_TRUE(setSubnormalToZero(&t2));
		compareMatrices(Transform::Identity().matrix(), transform.matrix());
	}
	{
		Transform t3 = transform;
		t3(0, 1) = std::numeric_limits<Scalar>::quiet_NaN();
		Transform t4 = t3;
		EXPECT_FALSE(setSubnormalToZero(&t4));
		compareMatrices(t3.matrix(), t4.matrix());
	}
	{
		Transform t5 = transform;
		t5(0, 1) = std::numeric_limits<Scalar>::quiet_NaN();
		t5(1, 0) = 0;
		Transform t6 = t5;
		t6(1, 0) = std::numeric_limits<Scalar>::denorm_min();
		EXPECT_TRUE(setSubnormalToZero(&t6));
		compareMatrices(t5.matrix(), t6.matrix());
	}
}

TYPED_TEST(ValidTests, ClearSubnormalTransform)
{
	typedef typename TestFixture::Scalar Scalar;

	transformSetSubnormalHelper<Eigen::Transform<Scalar, 2, Eigen::Isometry>>();
	transformSetSubnormalHelper<Eigen::Transform<Scalar, 3, Eigen::Isometry>>();
	transformSetSubnormalHelper<Eigen::Transform<Scalar, 4, Eigen::Isometry>>();
	transformSetSubnormalHelper<Eigen::Transform<Scalar, 4, Eigen::Isometry>>();
	transformSetSubnormalHelper<Eigen::Transform<Scalar, 4, Eigen::Affine>>();
	transformSetSubnormalHelper<Eigen::Transform<Scalar, 4, Eigen::AffineCompact>>();
}

TYPED_TEST(ValidTests, Blocks)
{
	typedef typename TestFixture::Scalar Scalar;
	using SurgSim::Math::isValid;
	using SurgSim::Math::isSubnormal;
	using SurgSim::Math::setSubnormalToZero;

	{
		Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor> matrix;
		matrix.setConstant(123);
		EXPECT_TRUE(isValid(matrix.template block<2, 2>(1, 1)));
		EXPECT_FALSE(isSubnormal(matrix.template block<2, 2>(1, 1)));
		{
			auto submatrix = matrix.template block<2, 2>(1, 1);
			EXPECT_FALSE(setSubnormalToZero(&submatrix));
		}
	}
	{
		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> matrix;
		matrix.setConstant(11, 13, static_cast<Scalar>(123));
		EXPECT_TRUE(isValid(matrix.template block<6, 6>(3, 5)));
		EXPECT_FALSE(isSubnormal(matrix.template block<6, 6>(3, 5)));
		{
			auto submatrix = matrix.template block<6, 6>(3, 5);
			EXPECT_FALSE(setSubnormalToZero(&submatrix));
		}
	}
}
