// This file is a part of the OpenSurgSim project.
// Copyright 2012-2015, SimQuest Solutions Inc.
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
/// Definitions of small fixed-size vector types.

#ifndef SURGSIM_MATH_VECTOR_H
#define SURGSIM_MATH_VECTOR_H

#include <array>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{
namespace Math
{

/// A 2D vector of floats.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<float,  2, 1>  Vector2f;

/// A 3D vector of floats.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<float,  3, 1>  Vector3f;

/// A 4D vector of floats.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<float,  4, 1>  Vector4f;

/// A 6D vector of floats.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<float,  6, 1>  Vector6f;

/// A 2D vector of doubles.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 2, 1>  Vector2d;

/// A 3D vector of doubles.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 3, 1>  Vector3d;

/// A 4D vector of doubles.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 4, 1>  Vector4d;

/// A 6D matrix of doubles.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/// A dynamic size column vector
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;

/// Helper method to add a sub-vector into a vector, for the sake of clarity
/// \tparam Vector The vector type
/// \tparam SubVector The sub-vector type
/// \param subVector The sub-vector
/// \param blockId The block index in vector
/// \param blockSize The block size
/// \param[out] vector The vector to add the sub-vector into
template <class Vector, class SubVector>
void addSubVector(const SubVector& subVector, Eigen::Index blockId, Eigen::Index blockSize, Vector* vector)
{
	vector->segment(blockSize * blockId, blockSize) += subVector;
}

/// Helper method to add a sub-vector per block into a vector, for the sake of clarity
/// \tparam VectorType The vector type
/// \param subVector The sub-vector (containing all the blocks)
/// \param blockIds Vector of block indices (for accessing vector) corresponding to the blocks in sub-vector
/// \param blockSize The block size
/// \param[out] vector The vector to add the sub-vector blocks into
template <class VectorType>
void addSubVector(const Eigen::Ref<const Vector>& subVector,
	const std::vector<size_t>& blockIds, Eigen::Index blockSize, VectorType* vector)
{
	const Eigen::Index numBlocks = static_cast<Eigen::Index>(blockIds.size());
	for (Eigen::Index block = 0; block < numBlocks; ++block)
	{
		vector->segment(blockSize * static_cast<Eigen::Index>(blockIds[block]), blockSize) +=
			subVector.segment(blockSize * block, blockSize);
	}
}

/// Helper method to set a sub-vector into a vector, for the sake of clarity
/// \tparam Vector The vector type
/// \tparam SubVector The sub-vector type
/// \param subVector The sub-vector
/// \param blockId The block index in vector
/// \param blockSize The size of the sub-vector
/// \param[out] vector The vector to set the sub-vector into
template <class Vector, class SubVector>
void setSubVector(const SubVector& subVector, Eigen::Index blockId, Eigen::Index blockSize, Vector* vector)
{
	vector->segment(blockSize * blockId, blockSize) = subVector;
}

/// Helper method to access a sub-vector from a vector, for the sake of clarity
/// \tparam Vector The vector type to get the sub-vector from
/// \param vector The vector to get the sub-vector from
/// \param blockId The block index
/// \param blockSize The block size
/// \return The requested sub-vector
/// \note Disable cpplint warnings for use of non-const reference
/// \note Eigen has a specific type for VectorBlock that we want to return with read/write access
/// \note therefore the Vector from which the VectorBlock is built from must not be const
template <class Vector>
Eigen::VectorBlock<Vector> getSubVector(Vector& vector, Eigen::Index blockId, Eigen::Index blockSize) // NOLINT
{
	return vector.segment(blockSize * blockId, blockSize);
}

/// Helper method to get a sub-vector per block from a vector, for the sake of clarity
/// \tparam Vector The vector type
/// \tparam SubVector The sub-vector type
/// \param vector The vector (containing the blocks in a sparse manner)
/// \param blockIds Vector of block indices (for accessing vector) corresponding to the blocks in vector
/// \param blockSize The block size
/// \param[out] subVector The sub-vector to store the requested blocks (blockIds) from vector into
template <class Vector, class SubVector>
void getSubVector(const Vector& vector, const std::vector<size_t>& blockIds, Eigen::Index blockSize,
	SubVector* subVector)
{
	const Eigen::Index numBlocks = static_cast<Eigen::Index>(blockIds.size());
	for (Eigen::Index block = 0; block < numBlocks; ++block)
	{
		subVector->segment(blockSize * block, blockSize) =
			vector.segment(blockSize * static_cast<Eigen::Index>(blockIds[block]), blockSize);
	}
}

/// Interpolate (slerp) between 2 vectors
/// \tparam T the numeric data type used for arguments and the return value.  Can usually be deduced.
/// \tparam size the size of the vectors.  Can be deduced.
/// \tparam TOpt the option flags (alignment etc.) used for the Vector arguments.  Can be deduced.
/// \param previous The starting vector (at time 0.0).
/// \param next The ending vector (at time 1.0).
/// \param t  The interpolation time requested. Within [0..1], although note bounds are not checked.
/// \returns the transform resulting in the slerp interpolation at time t.
/// \note t=0 => returns vector 'previous'
/// \note t=1 => returns vector 'next'
template <typename T, int size, int TOpt>
Eigen::Matrix<T, size, 1, TOpt> interpolate(
	const Eigen::Matrix<T, size, 1, TOpt>& previous,
	const Eigen::Matrix<T, size, 1, TOpt>& next,
	T t)
{
	return previous + t * (next - previous);
}

/// Helper method to construct an orthonormal basis (i, j, k) given the 1st vector direction
/// \tparam T the numeric data type used for the vector argument. Can usually be deduced.
/// \tparam VOpt the option flags (alignment etc.) used for the vector argument. Can be deduced.
/// \param[in, out] i Should provide the 1st direction on input. The 1st vector of the basis (i, j, k) on output.
/// \param[out] j, k The 2nd and 3rd orthonormal vectors of the basis (i, j, k)
/// \return True if (i, j, k) has been built successfully, False if 'i' is a (or close to a) null vector
/// \note If any of the parameter is a nullptr, an exception will be raised
template <class T, int VOpt>
bool buildOrthonormalBasis(Eigen::Matrix<T, 3, 1, VOpt>* i,
						   Eigen::Matrix<T, 3, 1, VOpt>* j,
						   Eigen::Matrix<T, 3, 1, VOpt>* k)
{
	SURGSIM_ASSERT(i != nullptr) << "Parameter [in, out] 'i' is a nullptr";
	SURGSIM_ASSERT(j != nullptr) << "Parameter [out] 'j' is a nullptr";
	SURGSIM_ASSERT(k != nullptr) << "Parameter [out] 'k' is a nullptr";

	if (i->isZero())
	{
		return false;
	}

	i->normalize();
	*j = i->unitOrthogonal();
	*k = i->cross(*j);

	return true;
}

/// Calculate the best unit normal we can find in the direction of pXq for one of the endpoints of q.
/// Try multiple arrangements of the end points to reduce the artifacts when three of the vertices may
/// be nearly collinear.
/// \param p segment p
/// \param q segment q
/// \param epsilon when the norm of p x q is above epsilon, the cross product is assumed to be valid.
/// return the normalized cross product of p x q
template <class T, int VOpt>
Eigen::Matrix<T, 3, 1, VOpt> robustCrossProduct(const std::array<Eigen::Matrix<T, 3, 1, VOpt>, 2>& p,
		const std::array<Eigen::Matrix<T, 3, 1, VOpt>, 2>& q,
		T epsilon)
{

	auto p0p1 = p[1] - p[0];
	auto p1q0 = q[0] - p[1];
	auto p0q0 = q[0] - p[0];
	auto p1q1 = q[1] - p[1];
	auto pXq = p0p1.cross(p1q0);
	auto norm = pXq.norm();
	if (norm < epsilon)
	{
		pXq = p0p1.cross(p0q0);
		norm = pXq.norm();
	}
	if (norm < epsilon)
	{
		pXq = p0p1.cross(p1q1);
		norm = pXq.norm();
	}
	pXq *= static_cast<T>(1.0 / norm);
	return pXq;
}

};  // namespace Math
};  // namespace SurgSim

#endif  // SURGSIM_MATH_VECTOR_H
