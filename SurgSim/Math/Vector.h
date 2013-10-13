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
	const unsigned int springNumNodes = blockIds.size();

	for (unsigned int springNodeId = 0; springNodeId < springNumNodes; springNodeId++)
	{
		unsigned int nodeId = blockIds[springNodeId];

		vector->segment(blockSize * nodeId, blockSize) += subVector.segment(blockSize * springNodeId, blockSize);
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
template <class Vector>
Eigen::VectorBlock<Vector> getSubVector(Vector& vector, unsigned int blockId, unsigned int blockSize)
{
	return vector.segment(blockSize * blockId, blockSize);
}

};  // namespace Math
};  // namespace SurgSim

#endif  // SURGSIM_MATH_VECTOR_H
