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

/// \file SparseMatrixTests.cpp
/// Tests that exercise the functionality of our sparse matrices, which come
/// straight from Eigen.

#include <tuple>

#include <gtest/gtest.h>

//#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/SparseMatrix.h"

using std::tuple;
using std::tuple_element;

template <size_t N> class TypeValue {
public:
	static const size_t value = N;
};
template <size_t N> const size_t TypeValue<N>::value;

template <typename Tuple>
class SparseMatrices : public ::testing::Test
{
public:
	typedef typename tuple_element<0, Tuple>::type T;
	static const int Opt = tuple_element<1, Tuple>::type::value;
	typedef typename tuple_element<2, Tuple>::type I;

	void SetUp() override
	{
		m_columnId = 6;
		m_rowId = 11;

		m_matrixTooSmall.resize(3, 3);

		m_matrixWithoutExtraCoefficients.resize(18, 18);
		m_matrixMissingCoefficients.resize(18, 18);
		m_matrixWithoutExtraCoefficientsExpected.resize(18, 18);
		for (I i = 0; i < m_rowId; i++)
		{
			for (I j = 0; j < m_columnId; j++)
			{
				m_matrixWithoutExtraCoefficients.insert(i, j) = 1.0;
				m_matrixMissingCoefficients.insert(i, j) = 1.0;
				m_matrixWithoutExtraCoefficientsExpected.insert(i, j) = 1.0;
			}
			for (I j = m_columnId + 4; j < 18; j++)
			{
				m_matrixWithoutExtraCoefficients.insert(i, j) = 1.0;
				m_matrixMissingCoefficients.insert(i, j) = 1.0;
				m_matrixWithoutExtraCoefficientsExpected.insert(i, j) = 1.0;
			}
		}
		for (I i = m_rowId; i < m_rowId + 4; i++)
		{
			for (I j = m_columnId; j < m_columnId + 4; j++)
			{
				m_matrixWithoutExtraCoefficients.insert(i , j) = 1.0;
				if (i % 2 == 0 || j % 2 == 0)
				{
					m_matrixMissingCoefficients.insert(i, j) = 1.0;
				}
				m_matrixWithoutExtraCoefficientsExpected.insert(i, j) =
					(i - m_rowId == 0 && j - m_columnId == 3 ? 2 :
					static_cast<T>(i - m_rowId + 1) * static_cast<T>(j - m_columnId + 1));
			}
		}
		for (I i = m_rowId + 4; i < 18; i++)
		{
			for (I j = 0; j < m_columnId; j++)
			{
				m_matrixWithoutExtraCoefficients.insert(i, j) = 1.0;
				m_matrixMissingCoefficients.insert(i, j) = 1.0;
				m_matrixWithoutExtraCoefficientsExpected.insert(i, j) = 1.0;
			}
			for (I j = m_columnId + 4; j < 18; j++)
			{
				m_matrixWithoutExtraCoefficients.insert(i, j) = 1.0;
				m_matrixMissingCoefficients.insert(i, j) = 1.0;
				m_matrixWithoutExtraCoefficientsExpected.insert(i, j) = 1.0;
			}
		}
		m_matrixWithoutExtraCoefficients.makeCompressed();
		m_matrixMissingCoefficients.makeCompressed();
		m_matrixWithoutExtraCoefficientsExpected.makeCompressed();

		m_matrixWithExtraCoefficients = m_matrixWithoutExtraCoefficients;
		m_matrixWithExtraCoefficientsExpected = m_matrixWithoutExtraCoefficientsExpected;
		addElementOnBlockRowsCols(&m_matrixWithExtraCoefficients);
		addElementOnBlockRowsCols(&m_matrixWithExtraCoefficientsExpected);
	}

	void addElementOnBlockRowsCols(Eigen::SparseMatrix<T, Opt, I>* m)
	{
		m->insert(m_rowId, m_columnId - 3) = 1.0;
		m->insert(m_rowId, m_columnId - 1) = 1.0;
		m->insert(m_rowId - 4, m_columnId) = 1.0;
		m->insert(m_rowId - 2, m_columnId) = 1.0;
		m->insert(m_rowId - 3, m_columnId + 1) = 1.0;
		m->makeCompressed();
	}

	template <class Derived>
	void TestSetWithoutSearch(const Derived& sub, bool subTooSmall= false, bool success = true)
	{
		using SurgSim::Math::setSubMatrixWithoutSearch;

		if (subTooSmall)
		{
			EXPECT_THROW(\
				(setSubMatrixWithoutSearch<4, 4, true>(sub, m_rowId, m_columnId, &m_matrixWithoutExtraCoefficients)),\
				SurgSim::Framework::AssertionFailure);
		}
		else
		{
			// No recipient specified
			EXPECT_THROW(\
				(setSubMatrixWithoutSearch<4, 4, true, Derived, T, Opt, I>(sub, m_rowId, m_columnId, nullptr)),\
				SurgSim::Framework::AssertionFailure);

			// Recipient too small
			EXPECT_THROW(\
				(setSubMatrixWithoutSearch<4, 4, true>(sub, m_rowId, m_columnId, &m_matrixTooSmall)),\
				SurgSim::Framework::AssertionFailure);

			// Recipient does not have all the block coefficients (missing coefficients in the block)
			EXPECT_THROW(\
				(setSubMatrixWithoutSearch<4, 4, true>(sub, m_rowId, m_columnId, &m_matrixMissingCoefficients)),\
				SurgSim::Framework::AssertionFailure);

			// Recipient has extra coefficients on the block rows/columns
			EXPECT_THROW(\
				(setSubMatrixWithoutSearch<4, 4, true>(sub, m_rowId, m_columnId, &m_matrixWithExtraCoefficients)),\
				SurgSim::Framework::AssertionFailure);

			// Recipient is correct and sub is correct
			EXPECT_NO_THROW(\
				(setSubMatrixWithoutSearch<4, 4, true>(sub, m_rowId, m_columnId, &m_matrixWithoutExtraCoefficients)));

			if (success)
			{
				EXPECT_TRUE(m_matrixWithoutExtraCoefficients.isApprox(m_matrixWithoutExtraCoefficientsExpected));
			}
			else
			{
				EXPECT_FALSE(m_matrixWithoutExtraCoefficients.isApprox(m_matrixWithoutExtraCoefficientsExpected));
			}
		}
	}

	template <class Derived>
	void TestSetWithSearch(const Derived& sub, bool subTooSmall= false, bool success = true)
	{
		using SurgSim::Math::setSubMatrixWithSearch;

		if (subTooSmall)
		{
			EXPECT_THROW(\
				(setSubMatrixWithSearch<4, 4, true>(sub, m_rowId, m_columnId, &m_matrixWithExtraCoefficients)),\
				SurgSim::Framework::AssertionFailure);
		}
		else
		{
			// No recipient specified
			EXPECT_THROW(\
				(setSubMatrixWithSearch<4, 4, true, Derived, T, Opt, I>(sub, m_rowId, m_columnId, nullptr)),\
				SurgSim::Framework::AssertionFailure);

			// Recipient too small
			EXPECT_THROW(\
				(setSubMatrixWithSearch<4, 4, true>(sub, m_rowId, m_columnId, &m_matrixTooSmall)),\
				SurgSim::Framework::AssertionFailure);

			// Recipient does not have all the block coefficients (missing coefficients in the block)
			EXPECT_THROW(\
				(setSubMatrixWithSearch<4, 4, true>(sub, m_rowId, m_columnId, &m_matrixMissingCoefficients)),\
				SurgSim::Framework::AssertionFailure);

			// Recipient is correct and sub is correct
			EXPECT_NO_THROW(\
				(setSubMatrixWithSearch<4, 4, true>(sub, m_rowId, m_columnId, &m_matrixWithExtraCoefficients)));

			if (success)
			{
				EXPECT_TRUE(m_matrixWithExtraCoefficients.isApprox(m_matrixWithExtraCoefficientsExpected));
			}
			else
			{
				EXPECT_FALSE(m_matrixWithExtraCoefficients.isApprox(m_matrixWithExtraCoefficientsExpected));
			}
		}
	}

	template <typename Derived>
	void TestSetSparseMatrixBlock(const Eigen::SparseMatrixBase<Derived>& sub)
	{
		using SurgSim::Math::setSparseMatrixBlock;

		// No recipient specified
		EXPECT_THROW((setSparseMatrixBlock<Derived, T, Opt, I>(sub, m_rowId, m_columnId, nullptr)), \
			SurgSim::Framework::AssertionFailure);

		// Recipient too small
		EXPECT_THROW((setSparseMatrixBlock(sub, m_rowId, m_columnId, &m_matrixTooSmall)), \
			SurgSim::Framework::AssertionFailure);

		// With a recipient that has all the coefficients and no other on the rows/columns
		EXPECT_NO_THROW((setSparseMatrixBlock(sub, m_rowId, m_columnId, &m_matrixWithoutExtraCoefficients)));
		EXPECT_TRUE((m_matrixWithoutExtraCoefficients.block(m_rowId, m_columnId, 4, 4).isApprox(sub)));
		EXPECT_TRUE(m_matrixWithoutExtraCoefficientsExpected.isApprox(m_matrixWithoutExtraCoefficients));

		// With a recipient that has all the coefficients and others on the rows/columns
		EXPECT_NO_THROW((setSparseMatrixBlock(sub, m_rowId, m_columnId, &m_matrixWithExtraCoefficients)));
		EXPECT_TRUE((m_matrixWithExtraCoefficients.block(m_rowId, m_columnId, 4, 4).isApprox(sub)));
		EXPECT_TRUE(m_matrixWithExtraCoefficientsExpected.isApprox(m_matrixWithExtraCoefficients));

		// With a recipient that does not have all the block coefficients (missing coefficients in the block)
		EXPECT_NO_THROW((setSparseMatrixBlock(sub, m_rowId, m_columnId, &m_matrixMissingCoefficients)));
		EXPECT_TRUE((m_matrixMissingCoefficients.block(m_rowId, m_columnId, 4, 4).isApprox(sub)));
		EXPECT_TRUE(m_matrixWithoutExtraCoefficientsExpected.isApprox(m_matrixMissingCoefficients));

		// With an empty recipient
		Eigen::SparseMatrix<T, Opt, I> m_matrixEmpty(18, 18);
		EXPECT_NO_THROW((setSparseMatrixBlock(sub, m_rowId, m_columnId, &m_matrixEmpty)));
		EXPECT_TRUE((m_matrixEmpty.block(m_rowId, m_columnId, 4, 4).isApprox(sub)));
	}

	template <typename Derived>
	void TestSetSparseMatrixSegment(const Eigen::SparseMatrixBase<Derived>& sub)
	{
		using SurgSim::Math::setSparseMatrixBlock;

		// With an empty recipient
		Eigen::SparseMatrix<T, Opt, I> m_matrixEmpty(18, 18);
		EXPECT_NO_THROW((setSparseMatrixBlock(sub, m_rowId, m_columnId, &m_matrixEmpty)));
		EXPECT_TRUE(m_matrixEmpty.block(m_rowId, m_columnId, sub.rows(), sub.cols()).isApprox(sub));
	}

	template <typename Derived>
	void TestAddSparseMatrixBlock(const Eigen::SparseMatrixBase<Derived>& sub)
	{
		using SurgSim::Math::addSparseMatrixBlock;

		typedef typename Derived::Scalar TSub;
		const int OptSub = Eigen::SparseMatrixBase<Derived>::IsRowMajor ? Eigen::RowMajor : Eigen::ColMajor;
		typedef typename Derived::Index ISub;

		Eigen::SparseMatrix<TSub, OptSub, ISub> one(sub.rows(), sub.cols());
		for(ISub i = 0; i < sub.rows(); i++)
		{
			for(ISub j = 0; j < sub.cols(); j++)
			{
				one.insert(i, j) = 1.0;
			}
		}
		one.makeCompressed();

		Eigen::SparseMatrix<T, Opt, I> m_matrixOne(18, 18);
		for(I i = 0; i < 18; i++)
		{
			for(I j = 0; j < 18; j++)
			{
				m_matrixOne.insert(i, j) = 1.0;
			}
		}
		m_matrixOne.makeCompressed();

		EXPECT_NO_THROW((addSparseMatrixBlock(sub, m_rowId, m_columnId, &m_matrixOne)));
		EXPECT_TRUE(m_matrixOne.block(m_rowId, m_columnId, sub.rows(), sub.cols()).isApprox(sub + one));
	}

	template <typename Derived>
	void TestAddSparseMatrixSegment(const Eigen::SparseMatrixBase<Derived>& sub)
	{
		using SurgSim::Math::addSparseMatrixBlock;

		typedef typename Derived::Scalar TSub;
		const int OptSub = Eigen::SparseMatrixBase<Derived>::IsRowMajor ? Eigen::RowMajor : Eigen::ColMajor;
		typedef typename Derived::Index ISub;

		Eigen::SparseVector<TSub, OptSub, ISub> one(sub.size());
		for(ISub i = 0; i < sub.size(); i++)
		{
			one.insert(i) = 1.0;
		}

		Eigen::SparseMatrix<T, Opt, I> m_matrixOne(18, 18);
		for(I i = 0; i < 18; i++)
		{
			for(I j = 0; j < 18; j++)
			{
				m_matrixOne.insert(i, j) = 1.0;
			}
		}
		m_matrixOne.makeCompressed();

		EXPECT_NO_THROW((addSparseMatrixBlock(sub, m_rowId, m_columnId, &m_matrixOne)));
		EXPECT_TRUE(m_matrixOne.block(m_rowId, m_columnId, sub.rows(), sub.cols()).isApprox(sub + one));
	}

	template <class T, int n, int m, int Opt>
	Eigen::Matrix<T, n, m, Opt> getStaticMatrix()
	{
		Eigen::Matrix<T, n, 1> vec1 = Eigen::Matrix<T, n, 1>::LinSpaced(1.0, n);
		Eigen::Matrix<T, 1, m> vec2 = Eigen::Matrix<T, 1, m>::LinSpaced(1.0, m);
		Eigen::Matrix<T, n, m, Opt> result = vec1 * vec2;
		if (m >= 4)
		{
			result(0, 3) = 2;
		}
		return result;
	}

	template <class T, int n, int m, int Opt>
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Opt> getDynamicMatrix()
	{
		Eigen::Matrix<T, Eigen::Dynamic, 1> vec1 = Eigen::Matrix<T, Eigen::Dynamic, 1>::LinSpaced(n, 1.0, n);
		Eigen::Matrix<T, 1, Eigen::Dynamic> vec2 = Eigen::Matrix<T, 1, Eigen::Dynamic>::LinSpaced(m, 1.0, m);
		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Opt> result = vec1 * vec2;
		if (m >= 4)
		{
			result(0, 3) = 2;
		}
		return result;
	}

	template <class T, int n, int m, int Opt>
	Eigen::SparseMatrix<T, Opt> getSparseMatrix()
	{
		typedef typename Eigen::SparseVector<T, Eigen::ColMajor>::Index Index1;
		typedef typename Eigen::SparseVector<T, Eigen::RowMajor>::Index Index2;

		Eigen::SparseVector<T, Eigen::ColMajor> vec1(n);
		Eigen::SparseVector<T, Eigen::RowMajor> vec2(m);
		for (Index1 i = 0; i < n; i++)
		{
			vec1.insert(i) = static_cast<T>(i+1);
		}
		for (Index2 i = 0; i < m; i++)
		{
			vec2.insert(i) = static_cast<T>(i+1);
		}

		Eigen::SparseMatrix<T, Opt> result(vec1 * vec2);
		if (m >= 4)
		{
			result.coeffRef(0, 3) = 2;
		}
		return result;
	}

	template <class T, int n, int Opt>
	Eigen::SparseVector<T, Opt> getSparseVector()
	{
		typedef typename Eigen::SparseVector<T, Opt>::Index Index;

		Eigen::SparseVector<T, Opt> vec(n);
		for (Index i = 0; i < n; i++)
		{
			vec.insert(i) = static_cast<T>(i+1);
		}

		return vec;
	}

protected:
	Eigen::SparseMatrix<T, Opt, I> m_matrixWithoutExtraCoefficients;
	Eigen::SparseMatrix<T, Opt, I> m_matrixWithoutExtraCoefficientsExpected;
	Eigen::SparseMatrix<T, Opt, I> m_matrixWithExtraCoefficients;
	Eigen::SparseMatrix<T, Opt, I> m_matrixWithExtraCoefficientsExpected;
	Eigen::SparseMatrix<T, Opt, I> m_matrixTooSmall;
	Eigen::SparseMatrix<T, Opt, I> m_matrixMissingCoefficients;

	I m_rowId, m_columnId;
};

template <typename Tuple>
const int SparseMatrices<Tuple>::Opt;

typedef ::testing::Types<
	tuple<double, TypeValue<Eigen::ColMajor>, int>,
	tuple<double, TypeValue<Eigen::RowMajor>, int>,
	tuple<double, TypeValue<Eigen::ColMajor>, ptrdiff_t>,
	tuple<double, TypeValue<Eigen::RowMajor>, ptrdiff_t>,
	tuple<float, TypeValue<Eigen::ColMajor>, int>,
	tuple<float, TypeValue<Eigen::RowMajor>, int>,
	tuple<float, TypeValue<Eigen::ColMajor>, ptrdiff_t>,
	tuple<float, TypeValue<Eigen::RowMajor>, ptrdiff_t>> MyTypes;
TYPED_TEST_CASE(SparseMatrices, MyTypes);

TYPED_TEST(SparseMatrices, setSubMatrixWithoutSearch)
{
	typedef typename tuple_element<0, TypeParam>::type T;
	const int Opt = tuple_element<1, TypeParam>::type::value;
	const int OtherOpt = (Opt == Eigen::ColMajor ? Eigen::RowMajor : Eigen::ColMajor);

	{
		SCOPED_TRACE("Test with static dense input sub-matrix");
		this->TestSetWithoutSearch(this->template getStaticMatrix<T, 3, 4, Opt>(), true); ///< Sub too small (1D)
		this->TestSetWithoutSearch(this->template getStaticMatrix<T, 3, 3, Opt>(), true); ///< Sub too small (2D)
		this->TestSetWithoutSearch(this->template getStaticMatrix<T, 4, 4, Opt>());
		this->TestSetWithoutSearch(this->template getStaticMatrix<T, 4, 4, OtherOpt>());
		this->TestSetWithoutSearch(this->template getStaticMatrix<T, 6, 4, Opt>()); ///< Sub larger (1D)
		this->TestSetWithoutSearch(this->template getStaticMatrix<T, 5, 6, Opt>()); ///< Sub larger (2D)
	}

	{
		SCOPED_TRACE("Test with dynamic dense input sub-matrix");
		this->TestSetWithoutSearch(this->template getDynamicMatrix<T, 3, 4, Opt>(), true); ///< Sub too small (1D)
		this->TestSetWithoutSearch(this->template getDynamicMatrix<T, 3, 3, Opt>(), true); ///< Sub too small (2D)
		this->TestSetWithoutSearch(this->template getDynamicMatrix<T, 4, 4, Opt>());
		this->TestSetWithoutSearch(this->template getDynamicMatrix<T, 4, 4, OtherOpt>());
		this->TestSetWithoutSearch(this->template getDynamicMatrix<T, 6, 4, Opt>()); ///< Sub larger (1D)
		this->TestSetWithoutSearch(this->template getDynamicMatrix<T, 5, 6, Opt>()); ///< Sub larger (2D)
	}

	{
		SCOPED_TRACE("Test with sparse input sub-matrix");
		this->TestSetWithoutSearch(this->template getSparseMatrix<T, 3, 4, Opt>(), true); ///< Sub too small (1D)
		this->TestSetWithoutSearch(this->template getSparseMatrix<T, 3, 3, Opt>(), true); ///< Sub too small (2D)
		this->TestSetWithoutSearch(this->template getSparseMatrix<T, 4, 4, Opt>());
		/// Wrong SparseMatrix alignment: in general, this may lead to Eigen failure (program exit).
		//this->TestSetWithoutSearch(this->template getSparseMatrix<T, 4, 4, OtherOpt>(), false, false);
		this->TestSetWithoutSearch(this->template getSparseMatrix<T, 6, 4, Opt>()); ///< Sub larger (1D)
		this->TestSetWithoutSearch(this->template getSparseMatrix<T, 5, 6, Opt>()); ///< Sub larger (2D)
	}
}

TYPED_TEST(SparseMatrices, setSubMatrixWithSearch)
{
	typedef typename tuple_element<0, TypeParam>::type T;
	const int Opt = tuple_element<1, TypeParam>::type::value;
	const int OtherOpt = (Opt == Eigen::ColMajor ? Eigen::RowMajor : Eigen::ColMajor);

	{
		SCOPED_TRACE("Test with static dense input sub-matrix");
		this->TestSetWithSearch(this->template getStaticMatrix<T, 3, 4, Opt>(), true); ///< Sub too small (1D)
		this->TestSetWithSearch(this->template getStaticMatrix<T, 3, 3, Opt>(), true); ///< Sub too small (2D)
		this->TestSetWithSearch(this->template getStaticMatrix<T, 4, 4, Opt>());
		this->TestSetWithSearch(this->template getStaticMatrix<T, 4, 4, OtherOpt>());
		this->TestSetWithSearch(this->template getStaticMatrix<T, 6, 4, Opt>()); ///< Sub larger (1D)
		this->TestSetWithSearch(this->template getStaticMatrix<T, 5, 6, Opt>()); ///< Sub larger (2D)
	}

	{
		SCOPED_TRACE("Test with dynamic dense input sub-matrix");
		this->TestSetWithSearch(this->template getDynamicMatrix<T, 3, 4, Opt>(), true); ///< Sub too small (1D)
		this->TestSetWithSearch(this->template getDynamicMatrix<T, 3, 3, Opt>(), true); ///< Sub too small (2D)
		this->TestSetWithSearch(this->template getDynamicMatrix<T, 4, 4, Opt>());
		this->TestSetWithSearch(this->template getDynamicMatrix<T, 4, 4, OtherOpt>());
		this->TestSetWithSearch(this->template getDynamicMatrix<T, 6, 4, Opt>()); ///< Sub larger (1D)
		this->TestSetWithSearch(this->template getDynamicMatrix<T, 5, 6, Opt>()); ///< Sub larger (2D)
	}

	{
		SCOPED_TRACE("Test with sparse input sub-matrix");
		this->TestSetWithSearch(this->template getSparseMatrix<T, 3, 4, Opt>(), true); ///< Sub too small (1D)
		this->TestSetWithSearch(this->template getSparseMatrix<T, 3, 3, Opt>(), true); ///< Sub too small (2D)
		this->TestSetWithSearch(this->template getSparseMatrix<T, 4, 4, Opt>());
		/// Wrong SparseMatrix alignment: in general, this may lead to Eigen failure (program exit).
		//this->TestSetWithSearch(this->template getSparseMatrix<T, 4, 4, OtherOpt>(), false, false);
		this->TestSetWithSearch(this->template getSparseMatrix<T, 6, 4, Opt>()); ///< Sub larger (1D)
		this->TestSetWithSearch(this->template getSparseMatrix<T, 5, 6, Opt>()); ///< Sub larger (2D)
	}
}

TYPED_TEST(SparseMatrices, setSparseMatrixBlockTest)
{
	typedef typename tuple_element<0, TypeParam>::type T;

	this->TestSetSparseMatrixBlock(this->template getSparseMatrix<T, 4, 4, Eigen::ColMajor>());
	this->TestSetSparseMatrixBlock(this->template getSparseMatrix<T, 4, 4, Eigen::RowMajor>());
	// Using Eigen expression
	this->TestSetSparseMatrixBlock(this->template getSparseMatrix<T, 6, 7, Eigen::ColMajor>().block(0, 0, 4, 4));
	this->TestSetSparseMatrixBlock(this->template getSparseMatrix<T, 6, 7, Eigen::RowMajor>().block(0, 0, 4, 4));

	this->TestSetSparseMatrixSegment(this->template getSparseVector<T, 4, Eigen::RowMajor>());
	this->TestSetSparseMatrixSegment(this->template getSparseVector<T, 4, Eigen::ColMajor>());
	// Using Eigen expression
	this->TestSetSparseMatrixSegment(this->template getSparseVector<T, 7, Eigen::ColMajor>().segment(0, 4));
	this->TestSetSparseMatrixSegment(this->template getSparseVector<T, 7, Eigen::RowMajor>().segment(0, 4));
}

TYPED_TEST(SparseMatrices, addSparseMatrixBlockTest)
{
	typedef typename tuple_element<0, TypeParam>::type T;

	this->TestAddSparseMatrixBlock(this->template getSparseMatrix<T, 4, 4, Eigen::ColMajor>());
	this->TestAddSparseMatrixBlock(this->template getSparseMatrix<T, 4, 4, Eigen::RowMajor>());
	// Using Eigen expression
	this->TestAddSparseMatrixBlock(this->template getSparseMatrix<T, 6, 7, Eigen::ColMajor>().block(0, 0, 4, 4));
	this->TestAddSparseMatrixBlock(this->template getSparseMatrix<T, 6, 7, Eigen::RowMajor>().block(0, 0, 4, 4));

	this->TestAddSparseMatrixSegment(this->template getSparseVector<T, 4, Eigen::RowMajor>());
	this->TestAddSparseMatrixSegment(this->template getSparseVector<T, 4, Eigen::ColMajor>());
	// Using Eigen expression
	this->TestAddSparseMatrixSegment(this->template getSparseVector<T, 7, Eigen::ColMajor>().segment(0, 4));
	this->TestAddSparseMatrixSegment(this->template getSparseVector<T, 7, Eigen::RowMajor>().segment(0, 4));
}
