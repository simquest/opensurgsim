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
/// osg::Matrix3d matrix;
/// osg::Vec3d result = rowVector * matrix;
/// \endcode
///
/// For the result to be the same, the OSG matrix must be the transpose of the Eigen matrix.
/// These conversions perform this transposition.

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

/// Convert 2x2 matrix of floats to OSG
inline osg::Matrix2 toOsg(const SurgSim::Math::Matrix22f& matrix)
{
	return osg::Matrix2(matrix(0, 0), matrix(1, 0), 
						 matrix(0, 1), matrix(1, 1));
}
/// Convert 2x2 matrix of doubles to OSG
/// Note that the OSG matrix stores the values as floats, so precision will be lost.
inline osg::Matrix2 toOsg(const SurgSim::Math::Matrix22d& matrix)
{
	return osg::Matrix2(matrix(0, 0), matrix(1, 0), 
						matrix(0, 1), matrix(1, 1));
}
/// Convert from OSG to either 2x2 matrix of floats or doubles
/// Note that the OSG matrix stores the values as floats.
/// \tparam	T	Value type (float or double)
template <typename T> inline
Eigen::Matrix<T, 2, 2, Eigen::DontAlign | Eigen::RowMajor> fromOsg(const osg::Matrix2& matrix)
{
	Eigen::Matrix<T, 2, 2, Eigen::DontAlign | Eigen::RowMajor> eigenMatrix;
	eigenMatrix << matrix(0, 0), matrix(1, 0),
				   matrix(0, 1), matrix(1, 1);
	return eigenMatrix;
}

/// Convert 3x3 matrix of floats to OSG
inline osg::Matrix3 toOsg(const SurgSim::Math::Matrix33f& matrix)
{
	return osg::Matrix3(matrix(0, 0), matrix(1, 0), matrix(2, 0),
						matrix(0, 1), matrix(1, 1), matrix(2, 1),
						matrix(0, 2), matrix(1, 2), matrix(2, 2));
}
/// Convert 3x3 matrix of doubles to OSG
/// Note that the OSG matrix stores the values as floats, so precision will be lost.
inline osg::Matrix3 toOsg(const SurgSim::Math::Matrix33d& matrix)
{
	return osg::Matrix3(matrix(0, 0), matrix(1, 0), matrix(2, 0),
						matrix(0, 1), matrix(1, 1), matrix(2, 1),
						matrix(0, 2), matrix(1, 2), matrix(2, 2));
}
/// Convert from OSG to either 3x3 matrix of floats or doubles
/// Note that the OSG matrix stores the values as floats.
/// \tparam	T	Value type (float or double)
template <typename T> inline
Eigen::Matrix<T, 3, 3, Eigen::DontAlign | Eigen::RowMajor> fromOsg(const osg::Matrix3& matrix)
{
	Eigen::Matrix<T, 3, 3, Eigen::DontAlign | Eigen::RowMajor> eigenMatrix;
	eigenMatrix << matrix(0, 0), matrix(1, 0), matrix(2, 0),
				   matrix(0, 1), matrix(1, 1), matrix(2, 1),
				   matrix(0, 2), matrix(1, 2), matrix(2, 2);
	return eigenMatrix;
}

/// Convert 4x4 matrix of floats to OSG
inline osg::Matrixf toOsg(const SurgSim::Math::Matrix44f& matrix)
{
	return osg::Matrixf(matrix(0, 0), matrix(1, 0), matrix(2, 0), matrix(3, 0),
						matrix(0, 1), matrix(1, 1), matrix(2, 1), matrix(3, 1),
						matrix(0, 2), matrix(1, 2), matrix(2, 2), matrix(3, 2),
						matrix(0, 3), matrix(1, 3), matrix(2, 3), matrix(3, 3));
}
/// Convert from OSG to 4x4 matrix of floats
inline SurgSim::Math::Matrix44f fromOsg(const osg::Matrixf& matrix)
{
	SurgSim::Math::Matrix44f eigenMatrix;
	eigenMatrix << matrix(0, 0), matrix(1, 0), matrix(2, 0), matrix(3, 0),
				   matrix(0, 1), matrix(1, 1), matrix(2, 1), matrix(3, 1),
				   matrix(0, 2), matrix(1, 2), matrix(2, 2), matrix(3, 2),
				   matrix(0, 3), matrix(1, 3), matrix(2, 3), matrix(3, 3);
	return eigenMatrix;
}

/// Convert 4x4 matrix of doubles to OSG
inline osg::Matrixd toOsg(const SurgSim::Math::Matrix44d& matrix)
{
	return osg::Matrixd(matrix(0, 0), matrix(1, 0), matrix(2, 0), matrix(3, 0),
						matrix(0, 1), matrix(1, 1), matrix(2, 1), matrix(3, 1),
						matrix(0, 2), matrix(1, 2), matrix(2, 2), matrix(3, 2),
						matrix(0, 3), matrix(1, 3), matrix(2, 3), matrix(3, 3));
}
/// Convert from OSG to 4x4 matrix of doubles
inline SurgSim::Math::Matrix44d fromOsg(const osg::Matrixd& matrix)
{
	SurgSim::Math::Matrix44d eigenMatrix;
	eigenMatrix << matrix(0, 0), matrix(1, 0), matrix(2, 0), matrix(3, 0),
				   matrix(0, 1), matrix(1, 1), matrix(2, 1), matrix(3, 1),
				   matrix(0, 2), matrix(1, 2), matrix(2, 2), matrix(3, 2),
				   matrix(0, 3), matrix(1, 3), matrix(2, 3), matrix(3, 3);
	return eigenMatrix;
}

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGMATRIXCONVERSIONS_H
