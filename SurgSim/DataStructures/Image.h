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

#ifndef SURGSIM_DATASTRUCTURES_IMAGE_H
#define SURGSIM_DATASTRUCTURES_IMAGE_H

#include <Eigen/Core>

namespace SurgSim
{

namespace DataStructures
{

/// A templated Image class
///
/// \tparam T the data type stored in the Image
template<class T>
class Image
{
public:
	/// Constructor
	/// \param width the image width
	/// \param height the image height
	/// \param channels the number of channels in the image
	Image(size_t width, size_t height, size_t channels);

	/// Copy constructor from a data pointer
	/// \param width the image width
	/// \param height the image height
	/// \param channels the number of channels in the image
	/// \param data pointer to the data to copy from
	Image(size_t width, size_t height, size_t channels, const T* const data);

	/// Copy constructor from another Image
	Image(const Image<T>& other);

	/// Destructor
	virtual ~Image();

	/// Assignment Operator
	/// \param other The Image to copy from
	/// \return The Image that was assigned into
	Image<T>& operator=(const Image<T>& other);

	/// Get the Image width
	/// \return the width
	size_t getWidth() const;

	/// Get the Image height
	/// \return the height
	size_t getHeight() const;

	/// Get the Image size
	/// \return the image size
	std::array<size_t, 2> getSize() const;

	/// Get the number of channels in this Image
	/// \return the number of channels
	size_t getNumChannels() const;

	/// Get the data in the channel as an eigen matrix
	/// \param the channel number
	/// \return an eigen matrix
	Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>, 0, Eigen::InnerStride<>> getChannel(size_t channel);

	/// Get the pointer to the data
	/// \return  the data
	T* const getData();

private:
	const std::array<size_t, 2> m_size;
	const size_t m_channels;
	T* const m_data;
};

typedef Image<float> ImageF;

}
}

#include "SurgSim/DataStructures/Image-inl.h"

#endif //SURGSIM_DATASTRUCTURES_IMAGE_H
