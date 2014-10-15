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

#include <array>
#include <memory>

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
	/// Default Constructor
	Image();

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

	/// Copy constructor
	/// \param other Image to copy from
	Image(const Image<T>& other);

	/// Move constructor
	/// \param other Image to move data from
	Image(Image<T>&& other);

	/// Destructor
	virtual ~Image();

	/// Assignment Operator
	/// \param other The Image to copy from
	/// \return The Image that was assigned into
	Image<T>& operator=(const Image<T>& other);

	/// Move Assignment Operator
	/// \param other The Image to move data from
	/// \return The Image that was assigned into
	Image<T>& operator=(Image<T>&& other);

	/// Get the Image width
	/// \return the width
	size_t getWidth() const;

	/// Get the Image height
	/// \return the height
	size_t getHeight() const;

	/// Get the Image size
	/// \return the image size as (width, height, channels)
	std::array<size_t, 3> getSize() const;

	/// Get the number of channels in this Image
	/// \return the number of channels
	size_t getNumChannels() const;

	/// Type of the channel returned by getChannel
	typedef Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>, 0, Eigen::InnerStride<>> ChannelType;

	/// Get the data in the channel as an eigen matrix
	/// \param channel the channel number
	/// \return an eigen matrix
	ChannelType getChannel(size_t channel);

	/// Get the pointer to the data
	/// \return  the data
	T* const getData();

	/// Get the pointer to the data, constant version
	/// \return  the data
	const T* const getData() const;

private:
	size_t m_width;
	size_t m_height;
	size_t m_channels;
	std::unique_ptr<T[]> m_data;
};

}
}

#include "SurgSim/DataStructures/Image-inl.h"

#endif //SURGSIM_DATASTRUCTURES_IMAGE_H
