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
/// Conversions to and from OSG matrix types
///
/// SurgSim's Eigen vectors are a single <b>column</b>, and matrix operations use <b>postfix</b> notation.
/// OSG vectors are a single <b>row</b> and matrix operations use <b>prefix</b> notation,
///
/// For example, with Eigen, one might write:
/// \code{.cpp}
/// Vector3d columnVector;
/// Matrix33d matrix;
/// Vector3d result = matrix * columnVector;
/// \endcode
///
/// However, with OSG, this should be written in the form:
/// \code{.cpp}
/// osg::Vec3d rowVector;
/// osg::Matrix3d columnMajorMatrix;
/// osg::Vec3d result = rowVector * columnMajorMatrix;
/// \endcode
///
/// For the result to be the same, the OSG matrix data must be interpreted as column-major.
/// These conversions handle that.

#ifndef SURGSIM_GRAPHICS_OSGMATRIXCONVERSIONS_H
#define SURGSIM_GRAPHICS_OSGMATRIXCONVERSIONS_H

#include <SurgSim/Math/Matrix.h>

#include <osg/Matrixf>
#include <osg/Matrixd>
#include <osg/Uniform>

namespace SurgSim
{

namespace Graphics
{

/// Convert a fixed-size 2x2 matrix to OSG.
/// Note that the OSG matrix stores the values as floats, so depending on the type of the input matrix,
/// precision may be lost.
template <typename T, int MOpt> inline
const osg::Matrix2 toOsg(const Eigen::Matrix<T, 2, 2, MOpt>& matrix)
{
	osg::Matrix2 osgMatrix;
	Eigen::Map<Eigen::Matrix<float, 2, 2, Eigen::ColMajor>> osgMatrixMap(osgMatrix.ptr());
	osgMatrixMap = matrix.template cast<float>();
	return osgMatrix;
}

/// Convert from OSG to a 2x2 matrix of either floats or doubles.
/// Note that the OSG matrix stores the values as floats.
/// \tparam	T	Value type (float or double)
template <typename T> inline
const Eigen::Matrix<T, 2, 2, Eigen::RowMajor> fromOsg(const osg::Matrix2& matrix)
{
	return Eigen::Map<const Eigen::Matrix<float, 2, 2, Eigen::ColMajor>>(matrix.ptr()).cast<T>();
}

/// Convert a fixed-size 3x3 matrix to OSG.
/// Note that the OSG matrix stores the values as floats, so depending on the type of the input matrix,
/// precision may be lost.
template <typename T, int MOpt> inline
const osg::Matrix3 toOsg(const Eigen::Matrix<T, 3, 3, MOpt>& matrix)
{
	osg::Matrix3 osgMatrix;
	Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::ColMajor>> osgMatrixMap(osgMatrix.ptr());
	osgMatrixMap = matrix.template cast<float>();
	return osgMatrix;
}

/// Convert from OSG to a 3x3 matrix of either floats or doubles.
/// Note that the OSG matrix stores the values as floats.
/// \tparam	T	Value type (float or double)
template <typename T> inline
const Eigen::Matrix<T, 3, 3, Eigen::RowMajor> fromOsg(const osg::Matrix3& matrix)
{
	return Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::ColMajor>>(matrix.ptr()).cast<T>();
}

/// Convert a fixed-size 4x4 matrix of floats to OSG.
template <int MOpt> inline
const osg::Matrixf toOsg(const Eigen::Matrix<float, 4, 4, MOpt>& matrix)
{
	osg::Matrixf osgMatrix;
	Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::ColMajor>>(osgMatrix.ptr()) = matrix;
	return osgMatrix;
}

/// Convert from OSG to a 4x4 matrix of floats.
inline const Eigen::Matrix<float, 4, 4, Eigen::RowMajor> fromOsg(const osg::Matrixf& matrix)
{
	return Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::ColMajor>>(matrix.ptr());
}

/// Convert a fixed-size 4x4 matrix of doubles to OSG.
template <int MOpt> inline
const osg::Matrixd toOsg(const Eigen::Matrix<double, 4, 4, MOpt>& matrix)
{
	osg::Matrixd osgMatrix;
	Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(osgMatrix.ptr()) = matrix;
	return osgMatrix;
}

/// Convert from OSG to a 4x4 matrix of doubles.
inline const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> fromOsg(const osg::Matrixd& matrix)
{
	return Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(matrix.ptr());
}

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGMATRIXCONVERSIONS_H
