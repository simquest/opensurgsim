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

#ifndef SURGSIM_DATASTRUCTURES_IMAGEBASE_INL_H
#define SURGSIM_DATASTRUCTURES_IMAGEBASE_INL_H

#include "SurgSim/Framework/Assert.h"


namespace SurgSim
{
namespace DataStructures
{

template<class T>
ImageBase<T>::~ImageBase()
{
}

template<class T>
Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> ImageBase<T>::operator()(size_t x, size_t y)
{
	SURGSIM_ASSERT(x < m_width) << "x is larger than the image width (" << x << " >= " << m_width << ")";
	SURGSIM_ASSERT(y < m_height) << "y is larger than the image height (" << y << " >= " << m_height << ")";
	return Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>>
		(getData() + m_channels * (x + y * m_width), m_channels);
}

template<class T>
Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>> ImageBase<T>::operator()(size_t x, size_t y) const
{
	SURGSIM_ASSERT(x < m_width) << "x is larger than the image width (" << x << " >= " << m_width << ")";
	SURGSIM_ASSERT(y < m_height) << "y is larger than the image height (" << y << " >= " << m_height << ")";
	return Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>>
		(getData() + m_channels * (x + y * m_width), m_channels);
}

template<class T>
Eigen::Map<typename ImageBase<T>::ChannelType, Eigen::Unaligned, Eigen::Stride<-1, -1>>
ImageBase<T>::getChannel(size_t index)
{
	SURGSIM_ASSERT(index < m_channels) << "Channel number is larger than the number of channels";
	Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> stride(m_width*m_channels, m_channels);
	return Eigen::Map<ChannelType, Eigen::Unaligned, Eigen::Stride<-1, -1>>
		(getData() + index, m_width, m_height, stride);
}

template<class T>
Eigen::Map<const typename ImageBase<T>::ChannelType, Eigen::Unaligned, Eigen::Stride<-1, -1>>
ImageBase<T>::getChannel(size_t index) const
{
	SURGSIM_ASSERT(index < m_channels) << "Channel number is larger than the number of channels";
	Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> stride(m_width*m_channels, m_channels);
	return Eigen::Map<const ChannelType, Eigen::Unaligned, Eigen::Stride<-1, -1>>
		(getData() + index, m_width, m_height, stride);
}

template<class T>
void ImageBase<T>::setChannel(size_t index, const Eigen::Ref<const ChannelType>& data)
{
	auto myChannel = getChannel(index);
	SURGSIM_ASSERT(myChannel.rows() == data.rows() && myChannel.cols() == data.cols())
		<< "Channel data must be of size " << myChannel.rows() << "x" << myChannel.cols() << ". "
		<< data.rows() << "x" << data.cols() << " data was provided.";
	myChannel = data;
}

template<class T>
Eigen::Map<typename ImageBase<T>::VectorType, Eigen::Unaligned> ImageBase<T>::getAsVector()
{
	return Eigen::Map<VectorType, Eigen::Unaligned>(getData(), m_width * m_height * m_channels);
}

template<class T>
Eigen::Map<const typename ImageBase<T>::VectorType, Eigen::Unaligned> ImageBase<T>::getAsVector() const
{
	return Eigen::Map<const VectorType, Eigen::Unaligned>(getData(), m_width * m_height * m_channels);
}

template<class T>
void ImageBase<T>::setAsVector(const Eigen::Ref<const VectorType>& data)
{
	auto myVector = getAsVector();
	SURGSIM_ASSERT(myVector.rows() == data.rows() && myVector.cols() == data.cols())
		<< "Vector must be of size " << myVector.rows() << "x" << myVector.cols() << ". "
		<< "A " << data.rows() << "x" << data.cols() << " vector was provided.";
	myVector = data;
}

template<class T>
size_t ImageBase<T>::getWidth() const
{
	return m_width;
}

template<class T>
size_t ImageBase<T>::getHeight() const
{
	return m_height;
}

template<class T>
std::array<size_t, 3> ImageBase<T>::getSize() const
{
	std::array<size_t, 3> size = {m_width, m_height, m_channels};
	return size;
}

template<class T>
size_t ImageBase<T>::getNumChannels() const
{
	return m_channels;
}

template<class T>
void ImageBase<T>::setSize(size_t width, size_t height, size_t channels)
{
	m_width = width;
	m_height = height;
	m_channels = channels;
}

};
};

#endif //SURGSIM_DATASTRUCTURES_IMAGEBASE_INL_H
