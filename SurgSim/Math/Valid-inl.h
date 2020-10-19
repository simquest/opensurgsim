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
/// Implementation of isValid(), isSubnormal() and setSubnormalToZero().

#ifndef SURGSIM_MATH_VALID_INL_H
#define SURGSIM_MATH_VALID_INL_H

#include <boost/math/special_functions/fpclassify.hpp>


namespace SurgSim
{
namespace Math
{

namespace internal
{

template <typename T, class V>
class PredicateAlwaysTrueVisitor
{
public:
	typedef typename T::Scalar Scalar;

	PredicateAlwaysTrueVisitor() : m_result(true)
	{
	}

	inline bool getResult() const
	{
		return m_result;
	}

	inline void init(const Scalar& value, Eigen::Index i, Eigen::Index j)
	{
		if (! V::evaluate(value))
		{
			m_result = false;
		}
	}

	inline void operator()(const Scalar& value, Eigen::Index i, Eigen::Index j)
	{
		if (! V::evaluate(value))
		{
			m_result = false;
		}
	}

private:
	bool m_result;
};

template <typename T>
class ValidVisitor : public PredicateAlwaysTrueVisitor<T, ValidVisitor<T>>
{
public:
	typedef typename PredicateAlwaysTrueVisitor<T, ValidVisitor<T>>::Scalar Scalar;

	static bool evaluate(const Scalar& value)
	{
		// The extra parentheses protect from pain if isfinite() is also defined as a macro.
		return (boost::math::isfinite)(value);
	}
};

template <typename T>
class NonSubnormalVisitor : public PredicateAlwaysTrueVisitor<T, NonSubnormalVisitor<T>>
{
public:
	typedef typename PredicateAlwaysTrueVisitor<T, ValidVisitor<T>>::Scalar Scalar;

	static bool evaluate(const Scalar& value)
	{
		// The extra parentheses protect from pain if fpclassify() is also defined as a macro.
		return ((boost::math::fpclassify)(value) != FP_SUBNORMAL);
	}
};

};  // namespace internal


inline bool isValid(float value)
{
	// The extra parentheses protect from pain if isfinite() is also defined as a macro.
	return (boost::math::isfinite)(value);
}

inline bool isValid(double value)
{
	// The extra parentheses protect from pain if isfinite() is also defined as a macro.
	return (boost::math::isfinite)(value);
}

template <typename T>
inline bool isValid(const Eigen::DenseBase<T>& value)
{
	internal::ValidVisitor<T> visitor;
	value.visit(visitor);
	return visitor.getResult();
}

template <typename T>
inline bool isValid(const Eigen::QuaternionBase<T>& value)
{
	return isValid(value.coeffs());
}

template <typename T>
inline bool isValid(const Eigen::AngleAxis<T>& value)
{
	return isValid(value.angle()) && isValid(value.axis());
}

template <typename T>
inline bool isValid(const Eigen::Rotation2D<T>& value)
{
	return isValid(value.angle());
}

template <typename T, int D, int M, int O>
inline bool isValid(const Eigen::Transform<T, D, M, O>& value)
{
	return isValid(value.matrix());
}

inline bool isSubnormal(float value)
{
	// The extra parentheses protect from pain if fpclassify() is also defined as a macro.
	return ((boost::math::fpclassify)(value) == FP_SUBNORMAL);
}

inline bool isSubnormal(double value)
{
	// The extra parentheses protect from pain if fpclassify() is also defined as a macro.
	return ((boost::math::fpclassify)(value) == FP_SUBNORMAL);
}

template <typename T>
inline bool isSubnormal(const Eigen::DenseBase<T>& value)
{
	internal::NonSubnormalVisitor<T> visitor;
	value.visit(visitor);
	// The visitor checks whether *all* entries are "non-subnormal", i.e. false means some are subnormal.
	// This is a consequence of deriving from PredicateAlwaysTrueVisitor.
	return ! visitor.getResult();
}

template <typename T>
inline bool isSubnormal(const Eigen::QuaternionBase<T>& value)
{
	return isSubnormal(value.coeffs());
}

template <typename T>
inline bool isSubnormal(const Eigen::AngleAxis<T>& value)
{
	return isSubnormal(value.angle()) || isSubnormal(value.axis());
}

template <typename T>
inline bool isSubnormal(const Eigen::Rotation2D<T>& value)
{
	return isSubnormal(value.angle());
}

template <typename T, int D, int M, int O>
inline bool isSubnormal(const Eigen::Transform<T, D, M, O>& value)
{
	return isSubnormal(value.matrix());
}

inline bool setSubnormalToZero(float* value)
{
	// The extra parentheses protect from pain if fpclassify() is also defined as a macro.
	if ((boost::math::fpclassify)(*value) != FP_SUBNORMAL)
	{
		return false;
	}
	else
	{
		*value = 0.0f;
		return true;
	}
}

inline bool setSubnormalToZero(double* value)
{
	// The extra parentheses protect from pain if fpclassify() is also defined as a macro.
	if ((boost::math::fpclassify)(*value) != FP_SUBNORMAL)
	{
		return false;
	}
	else
	{
		*value = 0.0;
		return true;
	}
}

template <typename T>
inline bool setSubnormalToZero(Eigen::DenseBase<T>* value)
{
	if (! isSubnormal(*value))  // optimize for the common case where nothing is subnormal
	{
		return false;
	}

	// TODO(bert): This is a simple implementation; it could be much more optimized by e.g. unrolling loops along
	// the lines of the implementation of Eigen::DenseBase<T>::visit().  Unfortunately, we can't just *use* Eigen's
	// visitors here, because the visitor API doesn't allow modifying the values.

	using Eigen::Index;
	const Index numColumns = value->cols();
	const Index numRows = value->rows();

	for (Index j = 0; j < numColumns; ++j)
	{
		for (Index i = 0; i < numRows; ++i)
		{
			// The extra parentheses protect from pain if fpclassify() is also defined as a macro.
			if ((boost::math::fpclassify)(value->coeffRef(i, j)) == FP_SUBNORMAL)
			{
				value->coeffRef(i, j) = 0;
			}
		}
	}
	return true;
}

template <typename T>
inline bool setSubnormalToZero(Eigen::QuaternionBase<T>* value)
{
	return setSubnormalToZero(&(value->coeffs()));
}

template <typename T>
inline bool setSubnormalToZero(Eigen::AngleAxis<T>* value)
{
	bool angleChanged = setSubnormalToZero(&(value->angle()));
	bool axisChanged = setSubnormalToZero(&(value->axis()));
	return angleChanged || axisChanged;  // careful about short-circuiting!
}

template <typename T>
inline bool setSubnormalToZero(Eigen::Rotation2D<T>* value)
{
	return setSubnormalToZero(&(value->angle()));
}

template <typename T, int D, int M, int O>
inline bool setSubnormalToZero(Eigen::Transform<T, D, M, O>* value)
{
	return setSubnormalToZero(&(value->matrix()));
}

};  // namespace Math
};  // namespace SurgSim

#endif  // SURGSIM_MATH_VALID_INL_H
