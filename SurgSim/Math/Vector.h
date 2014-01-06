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
/// Definitions of small fixed-size vector types.

#ifndef SURGSIM_MATH_VECTOR_H
#define SURGSIM_MATH_VECTOR_H

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
typedef Eigen::Matrix<float,  2, 1, Eigen::DontAlign>  Vector2f;

/// A 3D vector of floats.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<float,  3, 1, Eigen::DontAlign>  Vector3f;

/// A 4D vector of floats.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<float,  4, 1, Eigen::DontAlign>  Vector4f;

/// A 2D vector of doubles.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign>  Vector2d;

/// A 3D vector of doubles.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign>  Vector3d;

/// A 4D vector of doubles.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 4, 1, Eigen::DontAlign>  Vector4d;

/// A dynamic size column vector
typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign> Vector;

/// Helper method to add a sub-vector into a vector, for the sake of clarity
/// \tparam Vector The vector type
/// \tparam SubVector The sub-vector type
/// \param subVector The sub-vector
/// \param blockId The block index in vector
/// \param blockSize The block size
/// \param[out] vector The vector to add the sub-vector into
template <class Vector, class SubVector>
void addSubVector(const SubVector& subVector,unsigned int blockId, unsigned int blockSize, Vector* vector)
{
	vector->segment(blockSize * blockId, blockSize) += subVector;
}

/// Helper method to add a sub-vector per block into a vector, for the sake of clarity
/// \tparam Vector The vector type
/// \tparam SubVector The sub-vector type
/// \param subVector The sub-vector (containing all the blocks)
/// \param blockIds Vector of block indices (for accessing vector) corresponding to the blocks in sub-vector
/// \param blockSize The block size
/// \param[out] vector The vector to add the sub-vector blocks into
template <class Vector, class SubVector>
void addSubVector(const SubVector& subVector, const std::vector<unsigned int> blockIds,
	unsigned int blockSize, Vector* vector)
{
	const unsigned int numBlocks = blockIds.size();

	for (unsigned int block = 0; block < numBlocks; block++)
	{
		unsigned int blockId = blockIds[block];

		vector->segment(blockSize * blockId, blockSize) += subVector.segment(blockSize * block, blockSize);
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
void setSubVector(const SubVector& subVector,unsigned int blockId, unsigned int blockSize, Vector* vector)
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
Eigen::VectorBlock<Vector> getSubVector(Vector& vector, unsigned int blockId, unsigned int blockSize) // NOLINT
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
void getSubVector(const Vector& vector, const std::vector<unsigned int> blockIds,
	unsigned int blockSize, SubVector* subVector)
{
	const unsigned int numBlocks = blockIds.size();

	for (unsigned int block = 0; block < numBlocks; block++)
	{
		unsigned int blockId = blockIds[block];

		subVector->segment(blockSize * block, blockSize) = vector.segment(blockSize * blockId, blockSize);
	}
}

/// Helper method to resize a vector (if necessary), and potentially zero it out
/// \tparam Vector The vector type
/// \param[in,out] v The vector to resize and potentially zero out
/// \param size The size to resize the vector v to
/// \param zeroOut True if the vector v should be filled up with 0 after having been resized, False if not
template <class Vector>
void resize(Vector *v, unsigned int size, bool zeroOut = false)
{
	if (v == nullptr)
	{
		return;
	}
	if (v->size() != static_cast<int>(size))
	{
		v->resize(static_cast<int>(size));
	}
	if (zeroOut)
	{
		v->setZero();
	}
}

template <typename T, int size, int TOpt>
Eigen::Matrix<T, size, 1, TOpt> interpolate(
	const Eigen::Matrix<T, size, 1, TOpt> &previous,
	const Eigen::Matrix<T, size, 1, TOpt> &next,
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
};  // namespace Math
};  // namespace SurgSim

#endif  // SURGSIM_MATH_VECTOR_H
