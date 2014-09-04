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
	std::copy(data, data+width*height*channels, m_data.get());
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
typename Image<T>::ChannelType Image<T>::getChannel(size_t channel)
{
	SURGSIM_ASSERT(channel < m_channels) << "channel number is larger than the number of channels";
	return ChannelType(m_data.get() + channel, m_width, m_height, Eigen::InnerStride<>(m_channels));
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
	return std::move(size);
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
