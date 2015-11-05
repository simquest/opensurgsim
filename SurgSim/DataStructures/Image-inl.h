// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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


#ifndef SURGSIM_DATASTRUCTURES_IMAGE_INL_H
#define SURGSIM_DATASTRUCTURES_IMAGE_INL_H

#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{
namespace DataStructures
{

template<class T>
Image<T>::Image() :
	m_width(0), m_height(0), m_channels(0)
{
}


template<class T>
Image<T>::Image(size_t width, size_t height, size_t channels) :
	m_width(width), m_height(height), m_channels(channels), m_data(new T[m_width * m_height * m_channels])
{
}

template<class T>
Image<T>::Image(size_t width, size_t height, size_t channels, const T* const data) :
	m_width(width), m_height(height), m_channels(channels), m_data(new T[m_width * m_height * m_channels])
{
	std::copy(data, data + width * height * channels, m_data.get());
}

template<class T>
template<class D>
Image<T>::Image(size_t width, size_t height, size_t channels, const D* const data) :
	m_width(width), m_height(height), m_channels(channels), m_data(new T[m_width * m_height * m_channels])
{
	Eigen::Map<const Eigen::Matrix<D, Eigen::Dynamic, 1>> theirData(data, width * height * channels);
	Eigen::Map<VectorType, Eigen::Unaligned> myData(m_data.get(), width * height * channels);
	myData = theirData.template cast<T>();
}

template<class T>
Image<T>::Image(const Image<T>& other) :
	m_width(other.getWidth()), m_height(other.getHeight()), m_channels(other.getNumChannels()),
	m_data(new T[m_width * m_height * m_channels])
{
	std::copy(other.m_data.get(), other.m_data.get() + m_width * m_height * m_channels, m_data.get());
}

template<class T>
Image<T>::Image(Image<T>&& other)
{
	// Can use the move assignment operator to construct
	*this = std::move(other);
}

template<class T>
Image<T>& Image<T>::operator=(const Image<T>& other)
{
	if (this != &other)
	{
		size_t newDataSize = other.getWidth() * other.getHeight() * other.getNumChannels();
		size_t oldDataSize = getWidth() * getHeight() * getNumChannels();
		if (newDataSize != oldDataSize)
		{
			m_data.reset(new T[newDataSize]);
		}
		m_width = other.getWidth();
		m_height = other.getHeight();
		m_channels = other.getNumChannels();
		std::copy(other.m_data.get(), other.m_data.get() + newDataSize, m_data.get());
	}
	return *this;
}

template<class T>
Image<T>& Image<T>::operator=(Image<T>&& other)
{
	if (this != &other)
	{
		m_data = std::move(other.m_data);
		m_width = other.getWidth();
		m_height = other.getHeight();
		m_channels = other.getNumChannels();

		other.m_width = 0;
		other.m_height = 0;
		other.m_channels = 0;
	}
	return *this;
}

template<class T>
Image<T>::~Image()
{
}

template<class T>
Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> Image<T>::operator()(size_t x, size_t y)
{
	SURGSIM_ASSERT(x < m_width) << "x is larger than the image width (" << x << " >= " << m_width << ")";
	SURGSIM_ASSERT(y < m_height) << "x is larger than the image height (" << y << " >= " << m_height << ")";
	return Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>>
		(m_data.get() + m_channels * (x + y * m_width), m_channels);
}

template<class T>
Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>> Image<T>::operator()(size_t x, size_t y) const
{
	SURGSIM_ASSERT(x < m_width) << "x is larger than the image width (" << x << " >= " << m_width << ")";
	SURGSIM_ASSERT(y < m_height) << "x is larger than the image height (" << y << " >= " << m_height << ")";
	return Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>>
		(m_data.get() + m_channels * (x + y * m_width), m_channels);
}

template<class T>
Eigen::Map<typename Image<T>::ChannelType, Eigen::Unaligned, Eigen::Stride<-1, -1>> Image<T>::getChannel(size_t index)
{
	SURGSIM_ASSERT(index < m_channels) << "Channel number is larger than the number of channels";
	Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> stride(m_width*m_channels, m_channels);
	return Eigen::Map<ChannelType, Eigen::Unaligned, Eigen::Stride<-1, -1>>
		(m_data.get() + index, m_width, m_height, stride);
}

template<class T>
Eigen::Map<const typename Image<T>::ChannelType, Eigen::Unaligned, Eigen::Stride<-1, -1>>
Image<T>::getChannel(size_t index) const
{
	SURGSIM_ASSERT(index < m_channels) << "Channel number is larger than the number of channels";
	Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> stride(m_width*m_channels, m_channels);
	return Eigen::Map<const ChannelType, Eigen::Unaligned, Eigen::Stride<-1, -1>>
		(m_data.get() + index, m_width, m_height, stride);
}

template<class T>
void Image<T>::setChannel(size_t index, const Eigen::Ref<const ChannelType>& data)
{
	auto myChannel = getChannel(index);
	SURGSIM_ASSERT(myChannel.rows() == data.rows() && myChannel.cols() == data.cols())
		<< "Channel data must be of size " << myChannel.rows() << "x" << myChannel.cols() << ". "
		<< data.rows() << "x" << data.cols() << " data was provided.";
	myChannel = data;
}

template<class T>
Eigen::Map<typename Image<T>::VectorType, Eigen::Unaligned> Image<T>::getAsVector()
{
	return Eigen::Map<VectorType, Eigen::Unaligned>(m_data.get(), m_width * m_height * m_channels);
}

template<class T>
Eigen::Map<const typename Image<T>::VectorType, Eigen::Unaligned> Image<T>::getAsVector() const
{
	return Eigen::Map<const VectorType, Eigen::Unaligned>(m_data.get(), m_width * m_height * m_channels);
}

template<class T>
void Image<T>::setAsVector(const Eigen::Ref<const VectorType>& data)
{
	auto myVector = getAsVector();
	SURGSIM_ASSERT(myVector.rows() == data.rows() && myVector.cols() == data.cols())
		<< "Vector must be of size " << myVector.rows() << "x" << myVector.cols() << ". "
		<< "A " << data.rows() << "x" << data.cols() << " vector was provided.";
	myVector = data;
}

template<class T>
size_t Image<T>::getWidth() const
{
	return m_width;
}

template<class T>
size_t Image<T>::getHeight() const
{
	return m_height;
}

template<class T>
std::array<size_t, 3> Image<T>::getSize() const
{
	std::array<size_t, 3> size = {m_width, m_height, m_channels};
	return size;
}

template<class T>
size_t Image<T>::getNumChannels() const
{
	return m_channels;
}

template<class T>
T* const Image<T>::getData()
{
	return m_data.get();
}

template<class T>
const T* const Image<T>::getData() const
{
	return m_data.get();
}

}
}

#endif //SURGSIM_DATASTRUCTURES_IMAGE_INL_H
