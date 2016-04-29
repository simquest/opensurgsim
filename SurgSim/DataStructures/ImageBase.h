// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_DATASTRUCTURES_IMAGEBASE_H
#define SURGSIM_DATASTRUCTURES_IMAGEBASE_H

#include <array>
#include <Eigen/Core>


namespace SurgSim
{
namespace DataStructures
{

/// Base class for Image-like classes
///
/// \tparam T the data type of the image data
template<class T>
class ImageBase
{
public:
	/// Destructor
	virtual ~ImageBase();

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

	/// Get the pixel value at (x, y)
	/// \param x The horizontal image position
	/// \param y The vertical image position
	/// \return mutable pixel value as Eigen::Map of size (channels x 1)
	Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> operator()(size_t x, size_t y);

	/// Get the pixel value at (x, y), constant version
	/// \param x The horizontal position in the image
	/// \param y The vertical position in the image
	/// \return constant pixel value as Eigen::Map of size (channels x 1)
	Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>> operator()(size_t x, size_t y) const;

	/// 2D Channel Type;
	typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> ChannelType;

	/// Get the 2D image channel data
	/// \param index the channel number
	/// \return mutable channel data as an Eigen::Map (can be used as an Eigen::Matrix)
	Eigen::Map<ChannelType, Eigen::Unaligned, Eigen::Stride<-1, -1>> getChannel(size_t index);

	/// Get the 2D image channel data, constant version
	/// \param index the channel number
	/// \return constant channel data as an Eigen::Map (can be used as an Eigen::Matrix)
	Eigen::Map<const ChannelType, Eigen::Unaligned, Eigen::Stride<-1, -1>> getChannel(size_t index) const;

	/// Set the image data in the channel
	/// \param index the channel number
	/// \param data the channel data as a compatible Eigen type
	void setChannel(size_t index, const Eigen::Ref<const ChannelType>& data);

	/// 1D Vector Type;
	typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorType;

	/// Get the data as a 1D Vector
	/// \return mutable 1D data as an Eigen::Map (can be used as an Eigen::Matrix)
	Eigen::Map<VectorType, Eigen::Unaligned> getAsVector();

	/// Get the data as a 1D Vector, constant version
	/// \return constant 1D data as an Eigen::Map (can be used as an Eigen::Matrix)
	Eigen::Map<const VectorType, Eigen::Unaligned> getAsVector() const;

	/// Set the image data as a 1D Vector
	/// \param data the data as a compatible Eigen vector type
	void setAsVector(const Eigen::Ref<const VectorType>& data);

	/// Get the pointer to the data
	/// \return  the data
	virtual T* const getData() = 0;

	/// Get the pointer to the data, constant version
	/// \return  the data
	virtual const T* const getData() const = 0;

protected:
	/// Set the Image size
	/// \param width the image width
	/// \param height the image height
	/// \param channels the number of channels in the image
	void setSize(size_t width, size_t height, size_t channels);

private:
	size_t m_width;
	size_t m_height;
	size_t m_channels;
};

};
};

#include "SurgSim/DataStructures/ImageBase-inl.h"

#endif //SURGSIM_DATASTRUCTURES_IMAGEBASE_H
