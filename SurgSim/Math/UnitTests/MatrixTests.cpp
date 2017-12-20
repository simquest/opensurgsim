// This file is a part of the OpenSurgSim project.
// Copyright 2012-2016, SimQuest Solutions Inc.
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
/// Tests that exercise the functionality of our matrix typedefs, which come
/// straight from Eigen.

#include <math.h>

#include <Eigen/Geometry>  // SurgSim/Math/Matrix.h by itself does not provide cross()
#include <gtest/gtest.h>

#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/Matrix.h"

// Define test fixture class templates.
// We don't really need fixtures as such, but the templatization encodes type.

template <class T>
class MatrixTestBase : public testing::Test
{
public:
	typedef T Scalar;
};



template <class T>
class Matrix22Tests : public MatrixTestBase<typename T::Scalar>
{
public:
	typedef T Matrix22;
};

// This used to contain aligned (via Eigen::AutoAlign) matrix type aliases, but we got rid of those.
typedef ::testing::Types<SurgSim::Math::Matrix22d,
		SurgSim::Math::Matrix22f> Matrix22Variants;
TYPED_TEST_CASE(Matrix22Tests, Matrix22Variants);


template <class T>
class Matrix33Tests : public MatrixTestBase<typename T::Scalar>
{
public:
	typedef T Matrix33;
};

// This used to contain aligned (via Eigen::AutoAlign) matrix type aliases, but we got rid of those.
typedef ::testing::Types<SurgSim::Math::Matrix33d,
		SurgSim::Math::Matrix33f> Matrix33Variants;
TYPED_TEST_CASE(Matrix33Tests, Matrix33Variants);


template <class T>
class Matrix44Tests : public MatrixTestBase<typename T::Scalar>
{
public:
	typedef T Matrix44;
};

// This used to contain aligned (via Eigen::AutoAlign) matrix type aliases, but we got rid of those.
typedef ::testing::Types<SurgSim::Math::Matrix44d,
		SurgSim::Math::Matrix44f> Matrix44Variants;
TYPED_TEST_CASE(Matrix44Tests, Matrix44Variants);



template <class T>
class AllMatrixTests : public MatrixTestBase<typename T::Scalar>
{
public:
	typedef T Matrix;
};

template <class T>
class AllDynamicMatrixTests : public MatrixTestBase<typename T::Scalar>
{
public:
	typedef T Matrix;
};

// This used to contain aligned (via Eigen::AutoAlign) matrix type aliases, but we got rid of those.
typedef ::testing::Types<SurgSim::Math::Matrix22d,
		SurgSim::Math::Matrix22f,
		SurgSim::Math::Matrix33d,
		SurgSim::Math::Matrix33f,
		SurgSim::Math::Matrix44d,
		SurgSim::Math::Matrix44f> AllMatrixVariants;
TYPED_TEST_CASE(AllMatrixTests, AllMatrixVariants);

typedef ::testing::Types<Eigen::MatrixXd,
		Eigen::MatrixXf,
		SurgSim::Math::Matrix> AllDynamicMatrixVariants;
TYPED_TEST_CASE(AllDynamicMatrixTests, AllDynamicMatrixVariants);

template <class T>
class UnalignedMatrixTests : public MatrixTestBase<typename T::Scalar>
{
public:
	typedef T Matrix;
};

template <class T>
class UnalignedDynamicMatrixTests : public MatrixTestBase<typename T::Scalar>
{
public:
	typedef T Matrix;
};

typedef ::testing::Types<SurgSim::Math::UnalignedMatrix22d,
		SurgSim::Math::UnalignedMatrix22f,
		SurgSim::Math::UnalignedMatrix33d,
		SurgSim::Math::UnalignedMatrix33f,
		SurgSim::Math::UnalignedMatrix44d,
		SurgSim::Math::UnalignedMatrix44f> UnalignedMatrixVariants;
TYPED_TEST_CASE(UnalignedMatrixTests, UnalignedMatrixVariants);

typedef ::testing::Types<Eigen::MatrixXd,
		Eigen::MatrixXf,
		SurgSim::Math::Matrix> UnalignedDynamicMatrixVariants;
TYPED_TEST_CASE(UnalignedDynamicMatrixTests, UnalignedDynamicMatrixVariants);


// Now we're ready to start testing...


// ==================== CONSTRUCTION & INITIALIZATION ====================

/// Test that matrices can be constructed.
TYPED_TEST(Matrix22Tests, CanConstruct)
{
	typedef typename TestFixture::Matrix22 Matrix22;

	// Warning: Eigen *does not* provide a 1-argument constructor that
	// initializes all elements to the same value!  If you do something like
	//    SurgSim::Math::Matrix22fu oneArg2fu(1.23f);
	// the argument is converted to an integral type, interpreted as a size,
	// and promptly ignored because the size is fixed.  Oops.
	// To generate a constant matrix, use Matrix22f::Constant(val).

	Matrix22 default2;
}

/// Test that matrices can be constructed.
TYPED_TEST(Matrix33Tests, CanConstruct)
{
	typedef typename TestFixture::Matrix33 Matrix33;

	// Warning: Eigen *does not* provide a 1-argument constructor that
	// initializes all elements to the same value!  If you do something like
	//    SurgSim::Math::Matrix22fu oneArg2fu(1.23f);
	// the argument is converted to an integral type, interpreted as a size,
	// and promptly ignored because the size is fixed.  Oops.
	// To generate a constant matrix, use Matrix22f::Constant(val).

	Matrix33 default3;
}

/// Test that matrices can be constructed.
TYPED_TEST(Matrix44Tests, CanConstruct)
{
	typedef typename TestFixture::Matrix44 Matrix44;

	// Warning: Eigen *does not* provide a 1-argument constructor that
	// initializes all elements to the same value!  If you do something like
	//    SurgSim::Math::Matrix22fu oneArg2fu(1.23f);
	// the argument is converted to an integral type, interpreted as a size,
	// and promptly ignored because the size is fixed.  Oops.
	// To generate a constant matrix, use Matrix22f::Constant(val).

	Matrix44 default4;
}

/// Test that the default constructor DOESN'T initialize matrices.
//
// Only test the non-vectorized versions.  Otherwise, we'd need to
// allocate memory in a way that guarantees Eigen-compatible alignment.
//
// TODO(bert): There is some Eigen flag that causes matrices and matrices to be
//   initialized after all!  We should check for that here.
TYPED_TEST(UnalignedMatrixTests, DefaultConstructorInitialization)
{
	typedef typename TestFixture::Matrix Matrix;
	const int SIZE = Matrix::RowsAtCompileTime;

	EXPECT_TRUE(SIZE >= 2 && SIZE <= 4);
	EXPECT_EQ(SIZE, Matrix::ColsAtCompileTime);

	// Allocate a buffer for the matrix type on stack, based on the size
	// of the object we're testing.  The object will be allocated inside
	// the buffer using the placement syntax for the new() operator.
	// Eigen's new operatore will attempt to align returned value on word sized
	// boundaries, so add 64 bytes to guarantee enough size.
	unsigned char buffer[sizeof(Matrix) + 64];

	{
		// Please don't write production (non-test) code that looks like this. =)
		memset(&buffer, 0xF0, sizeof(buffer));
		Matrix* matrix = new (&buffer) Matrix;
		for (int row = 0;  row < SIZE;  ++row)
		{
			for (int col = 0;  col < SIZE;  ++col)
			{
				EXPECT_NE(0.0f, (*matrix)(row, col)) << row << "," << col << " was NOT supposed to be zeroed.";
			}
		}
		// Destroying the object is a good idea, even if unnecessary here:
		matrix->Matrix::~Matrix();
	}
}

/// Test setting the matrix using the << syntax.
TYPED_TEST(Matrix22Tests, ShiftCommaInitialization)
{
	typedef typename TestFixture::Matrix22 Matrix22;

	Matrix22 matrix;
	// Initialize elements in order.  Do NOT put parentheses around the list!
	matrix <<
		   1.1f, 1.2f,
				 1.3f, 1.4f;
	for (int row = 0;  row < 2;  ++row)
	{
		for (int col = 0;  col < 2;  ++col)
		{
			EXPECT_NEAR(1.1 + 0.2 * row + 0.1 * col, matrix(row, col), 1e-6) <<
					row << "," << col << " wasn't properly initialized.";
		}
	}
}

/// Test setting the matrix using the << syntax.
TYPED_TEST(Matrix33Tests, ShiftCommaInitialization)
{
	typedef typename TestFixture::Matrix33 Matrix33;

	Matrix33 matrix;
	// Initialize elements in order.  Do NOT put parentheses around the list!
	matrix <<
		   1.1f, 1.2f, 1.3f,
				 1.4f, 1.5f, 1.6f,
				 1.7f, 1.8f, 1.9f;
	for (int row = 0;  row < 3;  ++row)
	{
		for (int col = 0;  col < 3;  ++col)
		{
			EXPECT_NEAR(1.1 + 0.3 * row + 0.1 * col, matrix(row, col), 1e-6) <<
					row << "," << col << " wasn't properly initialized.";
		}
	}
}

/// Test setting the matrix using the << syntax.
TYPED_TEST(Matrix44Tests, ShiftCommaInitialization)
{
	typedef typename TestFixture::Matrix44 Matrix44;

	Matrix44 matrix;
	// Initialize elements in order.  Do NOT put parentheses around the list!
	matrix <<
		   1.1f, 1.2f, 1.3f, 1.4f,
				 1.5f, 1.6f, 1.7f, 1.8f,
				 1.9f, 2.0f, 2.1f, 2.2f,
				 2.3f, 2.4f, 2.5f, 2.6f;
	for (int row = 0;  row < 4;  ++row)
	{
		for (int col = 0;  col < 4;  ++col)
		{
			EXPECT_NEAR(1.1 + 0.4 * row + 0.1 * col, matrix(row, col), 1e-6) <<
					row << "," << col << " wasn't properly initialized.";
		}
	}
}

/// Test getting a zero value usable in expressions.
TYPED_TEST(AllMatrixTests, ZeroValue)
{
	typedef typename TestFixture::Matrix Matrix;
	const int SIZE = Matrix::RowsAtCompileTime;

	Matrix matrix = 1000 * Matrix::Zero();
	for (int row = 0;  row < SIZE;  ++row)
	{
		for (int col = 0;  col < SIZE;  ++col)
		{
			EXPECT_NEAR(0.0, matrix(row, col), 1e-20) << row << "," << col << " wasn't properly initialized.";
		}
	}
}

/// Test setting matrices to 0.
TYPED_TEST(AllMatrixTests, SetToZero)
{
	typedef typename TestFixture::Matrix Matrix;
	const int SIZE = Matrix::RowsAtCompileTime;

	Matrix matrix;
	matrix.setZero();
	for (int row = 0;  row < SIZE;  ++row)
	{
		for (int col = 0;  col < SIZE;  ++col)
		{
			EXPECT_NEAR(0.0, matrix(row, col), 1e-20) << row << "," << col << " wasn't properly cleared.";
		}
	}
}

/// Test getting a constant value usable in expressions.
TYPED_TEST(AllMatrixTests, ConstantValue)
{
	typedef typename TestFixture::Matrix Matrix;
	const int SIZE = Matrix::RowsAtCompileTime;

	Matrix matrix = 2 * Matrix::Constant(0.5f);
	for (int row = 0;  row < SIZE;  ++row)
	{
		for (int col = 0;  col < SIZE;  ++col)
		{
			EXPECT_NEAR(1.0, matrix(row, col), 1e-6) << row << "," << col << " wasn't properly initialized.";
		}
	}
}

/// Test setting matrices to a constant.
TYPED_TEST(AllMatrixTests, SetToConstant)
{
	typedef typename TestFixture::Matrix Matrix;
	const int SIZE = Matrix::RowsAtCompileTime;

	Matrix matrix;
	matrix.setConstant(7.2f);
	for (int row = 0;  row < SIZE;  ++row)
	{
		for (int col = 0;  col < SIZE;  ++col)
		{
			EXPECT_NEAR(7.2, matrix(row, col), 1e-6) << row << "," << col << " wasn't properly initialized.";
		}
	}
}

/// Test initializing a row-major Eigen matrix from a float array.
TYPED_TEST(AllMatrixTests, InitializeRowMajorFromArray)
{
	typedef typename TestFixture::Matrix Matrix;
	typedef typename TestFixture::Scalar T;
	const int SIZE = Matrix::RowsAtCompileTime;

	typedef Eigen::Matrix < T, SIZE, SIZE, Eigen::AutoAlign | Eigen::RowMajor > RMatrix;

	// This array has more elements than we will need.
	// The element type must match the matrix!
	const T inputArray[18]  =
	{
		0.01f,  1.02f,  2.03f,  3.04f,  4.05f,  5.06f,  6.07f,  7.08f,  8.09f,
		9.10f, 10.11f, 11.12f, 12.13f, 13.14f, 14.15f, 15.16f, 16.17f, 17.18f
	};

	RMatrix matrix(inputArray);
	for (int row = 0;  row < SIZE;  ++row)
	{
		for (int col = 0;  col < SIZE;  ++col)
		{
			EXPECT_NEAR(0.01 + (row * SIZE + col) * 1.01, matrix(row, col), 1e-6) <<
					row << "," << col << " wasn't properly initialized.";
		}
	}
}

/// Test initializing a column-major Eigen matrix from a float array.
TYPED_TEST(AllMatrixTests, InitializeColumnMajorFromArray)
{
	typedef typename TestFixture::Matrix Matrix;
	typedef typename TestFixture::Scalar T;
	const int SIZE = Matrix::RowsAtCompileTime;

	typedef Eigen::Matrix < T, SIZE, SIZE, Eigen::AutoAlign | Eigen::ColMajor > CMatrix;

	// This array has more elements than we will need.
	// The element type must match the matrix!
	const T inputArray[18]  =
	{
		0.01f,  1.02f,  2.03f,  3.04f,  4.05f,  5.06f,  6.07f,  7.08f,  8.09f,
		9.10f, 10.11f, 11.12f, 12.13f, 13.14f, 14.15f, 15.16f, 16.17f, 17.18f
	};

	CMatrix matrix(inputArray);
	for (int row = 0;  row < SIZE;  ++row)
	{
		for (int col = 0;  col < SIZE;  ++col)
		{
			EXPECT_NEAR(0.01 + (col * SIZE + row) * 1.01, matrix(row, col), 1e-6) <<
					row << "," << col << " wasn't properly initialized.";
		}
	}
}

/// Test initializing from a float array.
/// Among other things, tests that our matrices are row-major.
TYPED_TEST(AllMatrixTests, InitializeFromArray)
{
	typedef typename TestFixture::Matrix Matrix;
	typedef typename TestFixture::Scalar T;
	const int SIZE = Matrix::RowsAtCompileTime;

	// This array has more elements than we will need.
	// The element type must match the matrix!
	const T inputArray[18]  =
	{
		0.01f,  1.02f,  2.03f,  3.04f,  4.05f,  5.06f,  6.07f,  7.08f,  8.09f,
		9.10f, 10.11f, 11.12f, 12.13f, 13.14f, 14.15f, 15.16f, 16.17f, 17.18f
	};

	Matrix matrix(inputArray);
	for (int row = 0;  row < SIZE;  ++row)
	{
		for (int col = 0;  col < SIZE;  ++col)
		{
			EXPECT_NEAR(0.01 + (row * SIZE + col) * 1.01, matrix(row, col), 1e-6) <<
					row << "," << col << " wasn't properly initialized.";
		}
	}
}

// Test conversion to and from yaml node
TYPED_TEST(AllMatrixTests, YamlConvert)
{
	typedef typename TestFixture::Matrix Matrix;
	typedef typename TestFixture::Scalar T;

	// This array has more elements than we will need.
	// The element type must match the matrix!
	const T inputArray[18]  =
	{
		0.01f,  1.02f,  2.03f,  3.04f,  4.05f,  5.06f,  6.07f,  7.08f,  8.09f,
		9.10f, 10.11f, 11.12f, 12.13f, 13.14f, 14.15f, 15.16f, 16.17f, 17.18f
	};

	Matrix matrix(inputArray);

	YAML::Node node;

	ASSERT_NO_THROW(node = matrix);

	EXPECT_TRUE(node.IsSequence());
	EXPECT_EQ(matrix.rows(), static_cast<typename Matrix::Index>(node.size()));

	ASSERT_NO_THROW({Matrix expected = node.as<Matrix>();});
	EXPECT_TRUE(matrix.isApprox(node.as<Matrix>()));
}

/// Test assignment.
TYPED_TEST(AllMatrixTests, Assign)
{
	typedef typename TestFixture::Matrix Matrix;
	const int SIZE = Matrix::RowsAtCompileTime;

	Matrix a = Matrix::Constant(6.0f);
	EXPECT_NEAR(6.0f * SIZE * SIZE, a.sum(), 1e-6);
	const Matrix b = Matrix::Constant(7.0f);
	EXPECT_NEAR(7.0f * SIZE * SIZE, b.sum(), 1e-6);
	a = b;
	EXPECT_NEAR(7.0f * SIZE * SIZE, a.sum(), 1e-6);
}

// ==================== ACCESS ====================

/// Access by rows and columns.
TYPED_TEST(AllMatrixTests, RowsAndColumns)
{
	typedef typename TestFixture::Matrix Matrix;
	typedef typename TestFixture::Scalar T;
	const int SIZE = Matrix::RowsAtCompileTime;

	typedef Eigen::Matrix<T, SIZE, 1> Vector;

	Matrix a = Matrix::Zero();
	for (int i = 0;  i < SIZE;  ++i)
	{
		Vector rowVector = Vector::Constant(i + 1.f);
		a.row(i) = rowVector;
	}

	for (int i = 0;  i < SIZE;  ++i)
	{
		Vector rowVector = a.row(i);
		EXPECT_NEAR((i + 1.f) * SIZE, rowVector.sum(), 1e-6);
		Vector columnVector = a.col(i);
		EXPECT_NEAR(SIZE * (SIZE + 1) / 2., columnVector.sum(), 1e-6);
	}

	for (int i = 0;  i < SIZE;  ++i)
	{
		Vector columnVector = Vector::Constant(i + 1.f);
		a.col(i) = columnVector;
	}

	for (int i = 0;  i < SIZE;  ++i)
	{
		Vector columnVector = a.col(i);
		EXPECT_NEAR((i + 1.f) * SIZE, columnVector.sum(), 1e-6);
		Vector rowVector = a.row(i);
		EXPECT_NEAR(SIZE * (SIZE + 1) / 2., rowVector.sum(), 1e-6);
	}
}

/// Access to the diagonal.
TYPED_TEST(AllMatrixTests, Diagonal)
{
	typedef typename TestFixture::Matrix Matrix;
	typedef typename TestFixture::Scalar T;
	const int SIZE = Matrix::RowsAtCompileTime;

	typedef Eigen::Matrix<T, SIZE, 1> Vector;

	Matrix a = Matrix::Zero();
	{
		Vector diagonalVector = Vector::Constant(2.f);
		a.diagonal() = diagonalVector;
	}
	EXPECT_NEAR(2.f * SIZE, a.sum(), 1e-6);

	Matrix b = Matrix::Identity();
	{
		EXPECT_NEAR(1.f * SIZE, b.diagonal().sum(), 1e-6);
	}
}

// ==================== REPRESENTATION CONVERSIONS ====================

/// Test setting quaternions from an angle/axis rotation.
TYPED_TEST(Matrix33Tests, FromAngleAxis)
{
	typedef typename TestFixture::Matrix33 Matrix33;
	typedef typename TestFixture::Scalar   T;

	typedef Eigen::Matrix<T, 3, 1> Vector3;

	T angle = 0.1f;
	Vector3 axis = Vector3::UnitZ();

	Matrix33 expectedMatrix;
	T sinAngle = std::sin(angle);
	T cosAngle = std::cos(angle);
	expectedMatrix <<
				   cosAngle, -sinAngle, 0,
							 sinAngle, cosAngle, 0,
							 0, 0, 1;

	using SurgSim::Math::makeRotationMatrix;

	Matrix33 matrix = makeRotationMatrix(angle, axis);
	EXPECT_NEAR(0, (matrix - expectedMatrix).norm(), 9e-6) << "The rotation matrix wasn't properly computed.";
}


/// Test extracting an angle/axis rotation from a quaternion.
TYPED_TEST(Matrix33Tests, ToAngleAxis)
{
	typedef typename TestFixture::Matrix33 Matrix33;
	typedef typename TestFixture::Scalar T;

	typedef Eigen::Matrix<T, 3, 1> Vector3;

	T angle = -0.1f;
	Vector3 axis = Vector3::UnitZ();

	Matrix33 matrix;
	T sinAngle = std::sin(angle);
	T cosAngle = std::cos(angle);
	matrix <<
		   cosAngle, -sinAngle, 0,
					 sinAngle, cosAngle, 0,
					 0, 0, 1;

	using SurgSim::Math::computeAngleAndAxis;
	using SurgSim::Math::computeAngle;

	T angle2;
	Vector3 axis2;
	computeAngleAndAxis(matrix, &angle2, &axis2);
	EXPECT_NEAR(-angle,    angle2,    1e-6) << "angle wasn't properly computed.";
	EXPECT_NEAR(-axis.x(), axis2.x(), 1e-6) << "X wasn't properly computed.";
	EXPECT_NEAR(-axis.y(), axis2.y(), 1e-6) << "Y wasn't properly computed.";
	EXPECT_NEAR(-axis.z(), axis2.z(), 1e-6) << "Y wasn't properly computed.";

	EXPECT_NEAR(-angle, computeAngle(matrix), 1e-6) << "angle wasn't properly computed by computeAngle().";
}

/// Test building a skew symmetric matrix from a vector
TYPED_TEST(Matrix33Tests, MakeSkewSymmetricMatrixTest)
{
	typedef typename TestFixture::Matrix33 Matrix33;
	typedef typename TestFixture::Scalar T;
	typedef Eigen::Matrix<T, 3, 1> Vector3;

	Vector3 v(static_cast<T>(0.3), static_cast<T>(-1.4), static_cast<T>(8.3));
	Matrix33 matrix = SurgSim::Math::makeSkewSymmetricMatrix(v);
	EXPECT_TRUE(matrix.diagonal().isZero());
	EXPECT_NEAR(static_cast<T>(0.3), matrix(2, 1), std::numeric_limits<T>::epsilon());
	EXPECT_NEAR(static_cast<T>(-0.3), matrix(1, 2), std::numeric_limits<T>::epsilon());
	EXPECT_NEAR(static_cast<T>(-1.4), matrix(0, 2), std::numeric_limits<T>::epsilon());
	EXPECT_NEAR(static_cast<T>(1.4), matrix(2, 0), std::numeric_limits<T>::epsilon());
	EXPECT_NEAR(static_cast<T>(8.3), matrix(1, 0), std::numeric_limits<T>::epsilon());
	EXPECT_NEAR(static_cast<T>(-8.3), matrix(0, 1), std::numeric_limits<T>::epsilon());
}

/// Test extracting a vector from a skew symmetric part of a matrix
TYPED_TEST(Matrix33Tests, SkewTest)
{
	typedef typename TestFixture::Scalar T;
	typedef Eigen::Matrix<T, 3, 1> Vector3;

	Vector3 vExpected(static_cast<T>(0.3), static_cast<T>(-1.4), static_cast<T>(8.3));
	Vector3 v = SurgSim::Math::skew(SurgSim::Math::makeSkewSymmetricMatrix(vExpected));
	EXPECT_TRUE(v.isApprox(vExpected));
}

// ==================== ARITHMETIC ====================

/// Negation (unary minus).
TYPED_TEST(AllMatrixTests, Negate)
{
	typedef typename TestFixture::Matrix Matrix;
	const int SIZE = Matrix::RowsAtCompileTime;

	Matrix a = Matrix::Constant(0.1f);
	EXPECT_NEAR(0.1 * SIZE * SIZE, a.sum(), 1e-6);
	Matrix b = -a;
	EXPECT_NEAR(-0.1 * SIZE * SIZE, b.sum(), 1e-6);
}

/// Addition.
TYPED_TEST(AllMatrixTests, Add)
{
	typedef typename TestFixture::Matrix Matrix;
	const int SIZE = Matrix::RowsAtCompileTime;

	Matrix a = Matrix::Constant(0.1f);
	EXPECT_NEAR(0.1 * SIZE * SIZE, a.sum(), 1e-6);
	Matrix b = a + Matrix::Ones() + a;
	EXPECT_NEAR(1.2 * SIZE * SIZE, b.sum(), 1e-6);
}

/// Subtraction.
TYPED_TEST(AllMatrixTests, Subtract)
{
	typedef typename TestFixture::Matrix Matrix;
	const int SIZE = Matrix::RowsAtCompileTime;

	Matrix a = Matrix::Constant(0.1f);
	EXPECT_NEAR(0.1 * SIZE * SIZE, a.sum(), 1e-6);
	Matrix b = Matrix::Ones() - a;
	EXPECT_NEAR(0.9 * SIZE * SIZE, b.sum(), 1e-6);
}

/// Incrementing by a value.
TYPED_TEST(AllMatrixTests, AddTo)
{
	typedef typename TestFixture::Matrix Matrix;
	const int SIZE = Matrix::RowsAtCompileTime;

	Matrix a = Matrix::Constant(0.1f);
	EXPECT_NEAR(0.1 * SIZE * SIZE, a.sum(), 1e-6);
	a += Matrix::Ones();
	EXPECT_NEAR(1.1 * SIZE * SIZE, a.sum(), 1e-6);
}

/// Decrementing by a value.
TYPED_TEST(AllMatrixTests, SubtractFrom)
{
	typedef typename TestFixture::Matrix Matrix;
	const int SIZE = Matrix::RowsAtCompileTime;

	Matrix a = Matrix::Constant(1.1f);
	EXPECT_NEAR(1.1 * SIZE * SIZE, a.sum(), 1e-6);
	a -= Matrix::Ones();
	EXPECT_NEAR(0.1 * SIZE * SIZE, a.sum(), 1e-6);
}

/// Matrix-scalar multiplication.
TYPED_TEST(AllMatrixTests, MultiplyMatrixScalar)
{
	typedef typename TestFixture::Matrix Matrix;

	Matrix a = Matrix::Random();
	Matrix b = a * 1.23f;
	EXPECT_NEAR(1.23 * a.sum(), b.sum(), 1e-6);
}

/// Scalar-matrix multiplication.
TYPED_TEST(AllMatrixTests, MultiplyScalarMatrix)
{
	typedef typename TestFixture::Matrix Matrix;

	Matrix a = Matrix::Random();
	Matrix b = 1.23f * a;
	EXPECT_NEAR(1.23 * a.sum(), b.sum(), 1e-6);
}

/// Division by scalar.
TYPED_TEST(AllMatrixTests, DivideScalar)
{
	typedef typename TestFixture::Matrix Matrix;

	Matrix a = Matrix::Random();
	Matrix b = a / 1.23f;
	EXPECT_NEAR(a.sum() / 1.23, b.sum(), 1e-6);
}

/// Matrix-vector multiplication.
TYPED_TEST(AllMatrixTests, MultiplyMatrixVector)
{
	typedef typename TestFixture::Matrix Matrix;
	typedef typename TestFixture::Scalar T;
	const int SIZE = Matrix::RowsAtCompileTime;

	typedef Eigen::Matrix<T, SIZE, 1> Vector;

	Matrix a = Matrix::Random();
	Vector v = Vector::Zero();
	v[0] = 1.f;
	Vector w = a * v;
	EXPECT_NEAR(a.col(0).sum(), w.sum(), 1e-6);
}

/// Matrix-matrix multiplication.
TYPED_TEST(AllMatrixTests, MultiplyMatrixMatrix)
{
	typedef typename TestFixture::Matrix Matrix;
	const int SIZE = Matrix::RowsAtCompileTime;

	Matrix a = Matrix::Random();
	Matrix I = Matrix::Identity();
	Matrix b = I * a * I;
	EXPECT_NEAR(0, (b - a).norm(), SIZE * SIZE * 1e-6);
}

/// Matrix inverse.
TYPED_TEST(AllMatrixTests, Inverse)
{
	typedef typename TestFixture::Matrix Matrix;
	const int SIZE = Matrix::RowsAtCompileTime;

	Matrix a = 2.0f * Matrix::Identity() + 0.5f * Matrix::Random();   // try to make an invertible matrix
	Matrix b = a.inverse();
	Matrix ab = a * b;
	EXPECT_NEAR(0, (ab - Matrix::Identity()).norm(), SIZE * SIZE * 1e-6);
	Matrix ba = b * a;
	EXPECT_NEAR(0, (ba - Matrix::Identity()).norm(), SIZE * SIZE * 1e-6);
}

/// Matrix transpose.
TYPED_TEST(AllMatrixTests, Transpose)
{
	typedef typename TestFixture::Matrix Matrix;
	const int SIZE = Matrix::RowsAtCompileTime;

	Matrix a;
	for (int row = 0;  row < SIZE;  ++row)
	{
		for (int col = 0;  col < SIZE;  ++col)
		{
			a(row, col) = 2.f * row - 3.f * col;
		}
	}

	// Note: DO NOT do things like "a = a.transpose()"; aliasing will result in an error.
	// You can use transposeInPlace(), or "a = a.transpose().eval()".

	Matrix b = a.transpose();
	for (int row = 0;  row < SIZE;  ++row)
	{
		for (int col = 0;  col < SIZE;  ++col)
		{
			EXPECT_NEAR(2.f * col - 3.f * row, b(row, col), 1e-6);
		}
	}
}

/// Component-wise multiplication.
TYPED_TEST(AllMatrixTests, ComponentwiseMultiply)
{
	typedef typename TestFixture::Matrix Matrix;

	Matrix a = Matrix::Random();
	Matrix b = Matrix::Identity();
	Matrix c = a.cwiseProduct(b);
	EXPECT_NEAR(0, c.sum() - c.diagonal().sum(), 1e-6);
	EXPECT_NEAR(0, (a - c).diagonal().sum(), 1e-6);
}

/// Component-wise division.
TYPED_TEST(AllMatrixTests, ComponentwiseDivide)
{
	typedef typename TestFixture::Matrix Matrix;

	Matrix a = Matrix::Random();
	Matrix b = Matrix::Constant(0.5f);
	Matrix c = a.cwiseQuotient(b);
	EXPECT_NEAR(a.sum() * 2, c.sum(), 1e-6);
}

/// Frobenius norm and its square.
TYPED_TEST(AllMatrixTests, FrobeniusNorm)
{
	typedef typename TestFixture::Matrix Matrix;
	typedef typename TestFixture::Scalar T;
	const int SIZE = Matrix::RowsAtCompileTime;

	Matrix m;
	T sumSquares = 0;
	for (int row = 0;  row < SIZE;  ++row)
	{
		for (int col = 0;  col < SIZE;  ++col)
		{
			T value = row + col + 11.0f;
			m(row, col) = value;
			sumSquares += value * value;
		}
	}

	EXPECT_NEAR(sumSquares, m.squaredNorm(), 1e-4);
	EXPECT_NEAR(sqrt(sumSquares), m.norm(), 1e-4);
}

/// L1 (Manhattan) norm and L_Infinity (largest absolute value) norm.
TYPED_TEST(AllMatrixTests, L1NormAndLInfNorm)
{
	typedef typename TestFixture::Matrix Matrix;
	typedef typename TestFixture::Scalar T;
	const int SIZE = Matrix::RowsAtCompileTime;

	Matrix m;
	T sumAbsolute = 0;
	for (int row = 0;  row < SIZE;  ++row)
	{
		for (int col = 0;  col < SIZE;  ++col)
		{
			T value = row + col - (SIZE + 0.6f);
			m(row, col) = value;
			// NOTE: DON'T use plain abs(), it may truncate to int!
			sumAbsolute += std::abs(value);
		}
	}
	// the function is set up so that the (0,0) corner will always have the biggest absolute value
	T maxAbsolute = (SIZE + 0.6f);

	Matrix n = -m;
	// Ugh, "template" is required to get this to parse properly.  This is
	// triggered because the test is a part of a template class; you don't
	// need to do this in a non-template context.
	EXPECT_NEAR(sumAbsolute, m.template lpNorm<1>(), 1e-4) << "m=" << m;
	EXPECT_NEAR(sumAbsolute, n.template lpNorm<1>(), 1e-4) << "n=" << n;
	EXPECT_NEAR(maxAbsolute, m.template lpNorm<Eigen::Infinity>(), 1e-4) << "m=" << m;
	EXPECT_NEAR(maxAbsolute, n.template lpNorm<Eigen::Infinity>(), 1e-4) << "n=" << m;
}

/// Minimum and maximum elements.
TYPED_TEST(AllMatrixTests, MinAndMax)
{
	typedef typename TestFixture::Matrix Matrix;
	typedef typename TestFixture::Scalar T;
	const int SIZE = Matrix::RowsAtCompileTime;

	const T inputArray[18]  =
	{
		0.01f,  1.02f,  2.03f,  3.04f,  4.05f,  5.06f,  6.07f,  7.08f,  8.09f,
		9.10f, 10.11f, 11.12f, 12.13f, 13.14f, 14.15f, 15.16f, 16.17f, 17.18f
	};

	Matrix m(inputArray);
	EXPECT_NEAR(inputArray[0], m.minCoeff(), 1e-6);
	EXPECT_NEAR(inputArray[SIZE * SIZE - 1], m.maxCoeff(), 1e-6);
}

/// Trace.
TYPED_TEST(AllMatrixTests, Trace)
{
	typedef typename TestFixture::Matrix Matrix;
	typedef typename TestFixture::Scalar T;

	Matrix a = Matrix::Random();
	T expectedTrace = a.diagonal().sum();
	EXPECT_NEAR(expectedTrace, a.trace(), 1e-6);
}

/// Determinant.
TYPED_TEST(AllMatrixTests, Determinant)
{
	typedef typename TestFixture::Matrix Matrix;
	const int SIZE = Matrix::RowsAtCompileTime;

	Matrix m = exp(1.f) * Matrix::Identity();
	EXPECT_NEAR(exp(1.*SIZE), m.determinant(), 1e-4);
}

/// Determinant (explicit 2x2 result).
TYPED_TEST(Matrix22Tests, Determinant22)
{
	typedef typename TestFixture::Matrix22 Matrix22;
	typedef typename TestFixture::Scalar   T;

	Matrix22 m = Matrix22::Random();
	T expectedDeterminant = m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0);
	EXPECT_NEAR(expectedDeterminant, m.determinant(), 1e-6);
}

/// Determinant (explicit 3x3 result).
TYPED_TEST(Matrix33Tests, Determinant33)
{
	typedef typename TestFixture::Matrix33 Matrix33;
	typedef typename TestFixture::Scalar   T;

	Matrix33 m = Matrix33::Random();
	T expectedDeterminant = m.row(0).cross(m.row(1)).dot(m.row(2));
	EXPECT_NEAR(expectedDeterminant, m.determinant(), 1e-6);
}

// ==================== SUBMATRICES (EXTENDING/SHRINKING) ====================

/// Extending matrices using the block<r,c>() syntax.
TYPED_TEST(Matrix22Tests, Extend2to3)
{
	typedef typename TestFixture::Matrix22 Matrix22;
	typedef typename TestFixture::Scalar  T;

	typedef Eigen::Matrix<T, 3, 3> Matrix33;

	Matrix22 matrix2;
	matrix2 <<
			1.1f, 1.2f,
				  1.3f, 1.4f;

	Matrix33 matrix3 = Matrix33::Identity();
	// Ugh, this is efficient but "template" is required to get it to parse
	// properly.  This is triggered because the test is a part of a template
	// class; you don't need to do this in a non-template context.
	matrix3.template block<2, 2>(0, 0) = matrix2;

	EXPECT_NEAR(6.0, matrix3.sum(), 1e-6) << "extending was incorrect: " << matrix3;
}

/// Extending matrices using the block(i,j,r,c) syntax.
TYPED_TEST(Matrix22Tests, DynamicExtend2to3)
{
	typedef typename TestFixture::Matrix22 Matrix22;
	typedef typename TestFixture::Scalar  T;

	typedef Eigen::Matrix<T, 3, 3> Matrix33;

	Matrix22 matrix2;
	matrix2 <<
			1.1f, 1.2f,
				  1.3f, 1.4f;

	Matrix33 matrix3 = Matrix33::Identity();
	matrix3.block(0, 0, 2, 2) = matrix2;

	EXPECT_NEAR(6.0, matrix3.sum(), 1e-6) << "extending was incorrect: " << matrix3;
}

/// Shrinking matrices using the block<r,c>() syntax.
TYPED_TEST(Matrix22Tests, Shrink3to2)
{
	typedef typename TestFixture::Matrix22 Matrix22;
	typedef typename TestFixture::Scalar  T;

	typedef Eigen::Matrix<T, 3, 3> Matrix33;

	Matrix33 matrix3;
	matrix3 <<
			1.1f, 1.2f, 1.3f,
				  1.4f, 1.5f, 1.6f,
				  1.7f, 1.8f, 1.9f;

	Matrix22 matrix2;
	// Ugh, this is efficient but "template" is required to get it to parse
	// properly.  This is triggered because the test is a part of a template
	// class; you don't need to do this in a non-template context.
	matrix2 = matrix3.template block<2, 2>(0, 0);

	EXPECT_NEAR(5.2, matrix2.sum(), 1e-6) << "shrinking was incorrect: " << matrix2;
}

/// Extending matrices using the block<r,c>() syntax.
TYPED_TEST(Matrix33Tests, Extend2to3)
{
	typedef typename TestFixture::Matrix33 Matrix33;
	typedef typename TestFixture::Scalar  T;

	typedef Eigen::Matrix<T, 2, 2> Matrix22;

	Matrix22 matrix2;
	matrix2 <<
			1.1f, 1.2f,
				  1.3f, 1.4f;

	Matrix33 matrix3 = Matrix33::Identity();
	// Ugh, this is efficient but "template" is required to get it to parse
	// properly.  This is triggered because the test is a part of a template
	// class; you don't need to do this in a non-template context.
	matrix3.template block<2, 2>(0, 0) = matrix2;

	EXPECT_NEAR(6.0, matrix3.sum(), 1e-6) << "extending was incorrect: " << matrix3;
}

/// Shrinking matrices using the block<r,c>() syntax.
TYPED_TEST(Matrix33Tests, Shrink3to2)
{
	typedef typename TestFixture::Matrix33 Matrix33;
	typedef typename TestFixture::Scalar  T;

	typedef Eigen::Matrix<T, 2, 2> Matrix22;

	Matrix33 matrix3;
	matrix3 <<
			1.1f, 1.2f, 1.3f,
				  1.4f, 1.5f, 1.6f,
				  1.7f, 1.8f, 1.9f;

	Matrix22 matrix2;
	// Ugh, this is efficient but "template" is required to get it to parse
	// properly.  This is triggered because the test is a part of a template
	// class; you don't need to do this in a non-template context.
	matrix2 = matrix3.template block<2, 2>(0, 0);

	EXPECT_NEAR(5.2, matrix2.sum(), 1e-6) << "shrinking was incorrect: " << matrix2;
}

/// Extending matrices using the block<r,c>() syntax.
TYPED_TEST(Matrix33Tests, Extend3to4)
{
	typedef typename TestFixture::Matrix33 Matrix33;
	typedef typename TestFixture::Scalar  T;

	typedef Eigen::Matrix<T, 4, 4> Matrix44;

	Matrix33 matrix3;
	matrix3 <<
			1.1f, 1.2f, 1.3f,
				  1.4f, 1.5f, 1.6f,
				  1.7f, 1.8f, 1.9f;

	Matrix44 matrix4 = Matrix44::Identity();
	// Ugh, this is efficient but "template" is required to get it to parse
	// properly.  This is triggered because the test is a part of a template
	// class; you don't need to do this in a non-template context.
	matrix4.template block<3, 3>(0, 0) = matrix3;

	EXPECT_NEAR(14.5, matrix4.sum(), 1e-6) << "extending was incorrect: " << matrix4;
}

/// Shrinking matrices using the block<r,c>() syntax.
TYPED_TEST(Matrix33Tests, Shrink4to3)
{
	typedef typename TestFixture::Matrix33 Matrix33;
	typedef typename TestFixture::Scalar  T;

	typedef Eigen::Matrix<T, 4, 4> Matrix44;

	Matrix44 matrix4;
	matrix4 <<
			1.1f, 1.2f, 1.3f, 1.4f,
				  1.5f, 1.6f, 1.7f, 1.8f,
				  1.9f, 2.0f, 2.1f, 2.2f,
				  2.3f, 2.4f, 2.5f, 2.6f;

	Matrix33 matrix3;
	// Ugh, this is efficient but "template" is required to get it to parse
	// properly.  This is triggered because the test is a part of a template
	// class; you don't need to do this in a non-template context.
	matrix3 = matrix4.template block<3, 3>(0, 0);

	EXPECT_NEAR(14.4, matrix3.sum(), 1e-6) << "shrinking was incorrect: " << matrix3;
}

/// Extending matrices using the block<r,c>() syntax.
TYPED_TEST(Matrix44Tests, Extend3to4)
{
	typedef typename TestFixture::Matrix44 Matrix44;
	typedef typename TestFixture::Scalar  T;

	typedef Eigen::Matrix<T, 3, 3> Matrix33;

	Matrix33 matrix3;
	matrix3 <<
			1.1f, 1.2f, 1.3f,
				  1.4f, 1.5f, 1.6f,
				  1.7f, 1.8f, 1.9f;

	Matrix44 matrix4 = Matrix44::Identity();
	// Ugh, this is efficient but "template" is required to get it to parse
	// properly.  This is triggered because the test is a part of a template
	// class; you don't need to do this in a non-template context.
	matrix4.template block<3, 3>(0, 0) = matrix3;

	EXPECT_NEAR(14.5, matrix4.sum(), 1e-6) << "extending was incorrect: " << matrix4;
}

/// Shrinking matrices using the block<r,c>() syntax.
TYPED_TEST(Matrix44Tests, Shrink4to3)
{
	typedef typename TestFixture::Matrix44 Matrix44;
	typedef typename TestFixture::Scalar  T;

	typedef Eigen::Matrix<T, 3, 3> Matrix33;

	Matrix44 matrix4;
	matrix4 <<
			1.1f, 1.2f, 1.3f, 1.4f,
				  1.5f, 1.6f, 1.7f, 1.8f,
				  1.9f, 2.0f, 2.1f, 2.2f,
				  2.3f, 2.4f, 2.5f, 2.6f;

	Matrix33 matrix3;
	// Ugh, this is efficient but "template" is required to get it to parse
	// properly.  This is triggered because the test is a part of a template
	// class; you don't need to do this in a non-template context.
	matrix3 = matrix4.template block<3, 3>(0, 0);

	EXPECT_NEAR(14.4, matrix3.sum(), 1e-6) << "shrinking was incorrect: " << matrix3;
}

// ==================== TYPE CONVERSION ====================

/// Typecasting matrices (double <-> float conversions).
TYPED_TEST(AllMatrixTests, TypeCasting)
{
	typedef typename TestFixture::Matrix Matrix;
	typedef typename TestFixture::Scalar T;
	const int SIZE = Matrix::RowsAtCompileTime;
	typedef Eigen::Matrix<double, SIZE, SIZE> Matrixd;
	typedef Eigen::Matrix<float,  SIZE, SIZE> Matrixf;

	Matrix a = Matrix::Random();
	T expectedSum = a.sum();

	// Ugh, "template" is required to get this to parse properly.  This is
	// triggered because the test is a part of a template class; you don't
	// need to do this in a non-template context.
	Matrixd d = a.template cast<double>();
	EXPECT_NEAR(expectedSum, d.sum(), 1e-6);
	Matrixf f = a.template cast<float>();
	EXPECT_NEAR(expectedSum, f.sum(), 1e-6);
}

// ==================== MISCELLANEOUS ====================

/// Reading from and writing to arrays or blocks of double/float in memory.
TYPED_TEST(AllMatrixTests, ArrayReadWrite)
{
	typedef typename TestFixture::Matrix Matrix;
	typedef typename TestFixture::Scalar T;
	const int NUM_ELEMENTS = Matrix::SizeAtCompileTime;


	const T inputArray[18]  =
	{
		0.01f,  1.02f,  2.03f,  3.04f,  4.05f,  5.06f,  6.07f,  7.08f,  8.09f,
		9.10f, 10.11f, 11.12f, 12.13f, 13.14f, 14.15f, 15.16f, 16.17f, 17.18f
	};
	T outputArray[18];

	Eigen::Map<const Matrix> inputMatrix(inputArray);
	Matrix a = inputMatrix;

	Eigen::Map<Matrix> outputMatrix(outputArray);
	outputMatrix = a;

	for (int i = 0;  i < NUM_ELEMENTS;  ++i)
	{
		EXPECT_NEAR(inputArray[i], outputArray[i], 1e-6);
	}
}

// TO DO:
// non-checked access via coeff()
// testing numerical validity
// testing for denormalized numbers


namespace
{
template <class T>
void testScalar(T valueExpected, T value) {}

template <>
void testScalar<double>(double valueExpected, double value)
{
	EXPECT_DOUBLE_EQ(valueExpected, value);
}

template <>
void testScalar<float>(float valueExpected, float value)
{
	EXPECT_FLOAT_EQ(valueExpected, value);
}
};

TYPED_TEST(AllDynamicMatrixTests, addSubMatrix)
{
	typedef typename TestFixture::Matrix Matrix;

	Matrix m, mInit, m2, m2Init;
	m.resize(18, 18);
	m.setRandom();
	mInit  = m;
	m2.resize(18, 18);
	m2.setRandom();
	m2Init = m2;

	ASSERT_NO_THROW(SurgSim::Math::addSubMatrix(m2.block(3, 3, 3, 3), 2, 2, 3, 3, &m););

	EXPECT_TRUE(m2.isApprox(m2Init));
	EXPECT_FALSE(m.isApprox(mInit));
	for (int rowId = 0; rowId < 6; rowId++)
	{
		EXPECT_TRUE(m.row(rowId).isApprox(mInit.row(rowId)));
		EXPECT_TRUE(m.col(rowId).isApprox(mInit.col(rowId)));
	}
	for (int rowId = 6; rowId < 9; rowId++)
	{
		for (int colId = 6; colId < 9; colId++)
		{
			testScalar(mInit(rowId, colId) + m2Init(3 + rowId - 6, 3 + colId - 6), m(rowId, colId));
		}
	}
	for (int rowId = 9; rowId < 18; rowId++)
	{
		EXPECT_TRUE(m.row(rowId).isApprox(mInit.row(rowId)));
		EXPECT_TRUE(m.col(rowId).isApprox(mInit.col(rowId)));
	}
}

TYPED_TEST(AllDynamicMatrixTests, addSubMatrixBlocks)
{
	typedef typename TestFixture::Matrix Matrix;

	Matrix m, mInit, m2, m2Init;
	std::vector<size_t> nodeIds;
	m.resize(18, 18);
	m.setRandom();
	mInit = m;
	m2.resize(18, 18);
	m2.setRandom();
	m2Init = m2;
	nodeIds.push_back(1);
	nodeIds.push_back(3);
	nodeIds.push_back(5);

	ASSERT_NO_THROW(SurgSim::Math::addSubMatrix(m2.block(3, 3, 9, 9), nodeIds, 3, &m););
	EXPECT_TRUE(m2.isApprox(m2Init));
	EXPECT_FALSE(m.isApprox(mInit));
	for (int rowId = 0; rowId < 3; rowId++)
	{
		EXPECT_TRUE(m.row(rowId).isApprox(mInit.row(rowId)));
		EXPECT_TRUE(m.col(rowId).isApprox(mInit.col(rowId)));
	}
	for (int rowId = 3; rowId < 6; rowId++)
	{
		for (int colId = 3; colId < 6; colId++)
		{
			testScalar(mInit(rowId, colId) + m2Init(3 + rowId - 3, 3 + colId - 3), m(rowId, colId));
		}
		for (int colId = 9; colId < 12; colId++)
		{
			testScalar(mInit(rowId, colId) + m2Init(3 + rowId - 3, 3 + colId - 6), m(rowId, colId));
		}
		for (int colId = 15; colId < 18; colId++)
		{
			testScalar(mInit(rowId, colId) + m2Init(3 + rowId - 3, 3 + colId - 9), m(rowId, colId));
		}
	}
	for (int rowId = 6; rowId < 9; rowId++)
	{
		EXPECT_TRUE(m.row(rowId).isApprox(mInit.row(rowId)));
		EXPECT_TRUE(m.col(rowId).isApprox(mInit.col(rowId)));
	}
	for (int rowId = 9; rowId < 12; rowId++)
	{
		for (int colId = 3; colId < 6; colId++)
		{
			testScalar(mInit(rowId, colId) + m2Init(3 + rowId - 6, 3 + colId - 3), m(rowId, colId));
		}
		for (int colId = 9; colId < 12; colId++)
		{
			testScalar(mInit(rowId, colId) + m2Init(3 + rowId - 6, 3 + colId - 6), m(rowId, colId));
		}
		for (int colId = 15; colId < 18; colId++)
		{
			testScalar(mInit(rowId, colId) + m2Init(3 + rowId - 6, 3 + colId - 9), m(rowId, colId));
		}
	}
	for (int rowId = 12; rowId < 15; rowId++)
	{
		EXPECT_TRUE(m.row(rowId).isApprox(mInit.row(rowId)));
		EXPECT_TRUE(m.col(rowId).isApprox(mInit.col(rowId)));
	}
	for (int rowId = 15; rowId < 18; rowId++)
	{
		for (int colId = 3; colId < 6; colId++)
		{
			testScalar(mInit(rowId, colId) + m2Init(3 + rowId - 9, 3 + colId - 3), m(rowId, colId));
		}
		for (int colId = 9; colId < 12; colId++)
		{
			testScalar(mInit(rowId, colId) + m2Init(3 + rowId - 9, 3 + colId - 6), m(rowId, colId));
		}
		for (int colId = 15; colId < 18; colId++)
		{
			testScalar(mInit(rowId, colId) + m2Init(3 + rowId - 9, 3 + colId - 9), m(rowId, colId));
		}
	}
}

TYPED_TEST(AllDynamicMatrixTests, setSubMatrix)
{
	typedef typename TestFixture::Matrix Matrix;

	Matrix m, mInit, m2, m2Init;
	m.resize(18, 18);
	m.setRandom();
	mInit  = m;
	m2.resize(18, 18);
	m2.setRandom();
	m2Init = m2;

	ASSERT_NO_THROW(SurgSim::Math::setSubMatrix(m2.block(3, 3, 3, 3), 2, 2, 3, 3, &m););
	EXPECT_TRUE(m2.isApprox(m2Init));
	EXPECT_FALSE(m.isApprox(mInit));
	for (int rowId = 0; rowId < 6; rowId++)
	{
		EXPECT_TRUE(m.row(rowId).isApprox(mInit.row(rowId)));
		EXPECT_TRUE(m.col(rowId).isApprox(mInit.col(rowId)));
	}
	for (int rowId = 6; rowId < 6 + 3; rowId++)
	{
		for (int colId = 6; colId < 6 + 3; colId++)
		{
			testScalar(m2Init(3 + rowId - 6, 3 + colId - 6), m(rowId, colId));
		}
	}
	for (int rowId = 6 + 3; rowId < 18; rowId++)
	{
		EXPECT_TRUE(m.row(rowId).isApprox(mInit.row(rowId)));
		EXPECT_TRUE(m.col(rowId).isApprox(mInit.col(rowId)));
	}
}

TYPED_TEST(AllDynamicMatrixTests, getSubMatrix)
{
	typedef typename TestFixture::Matrix Matrix;

	Matrix m, mInit;
	m.resize(18, 18);
	m.setRandom();
	mInit = m;

	Eigen::Block<Matrix> subMatrix = SurgSim::Math::getSubMatrix(m, 2, 2, 3, 3);
	EXPECT_TRUE(m.isApprox(mInit));
	for (int rowId = 0; rowId < 3; rowId++)
	{
		for (int colId = 0; colId < 3; colId++)
		{
			testScalar(m(2 * 3 + rowId, 2 * 3 + colId), subMatrix(rowId, colId));
			// Also test that the returned value are pointing to the correct data
			EXPECT_EQ(&subMatrix(rowId, colId), &m(2 * 3 + rowId, 2 * 3 + colId));
		}
	}
}
