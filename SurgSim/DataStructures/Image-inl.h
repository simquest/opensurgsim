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
	m_size({{0, 0}}), m_channels(0), m_data(nullptr)
{
}


template<class T>
Image<T>::Image(size_t width, size_t height, size_t channels) :
	m_size({{width, height}}), m_channels(channels), m_data(new T[channels*width*height])
{
}

template<class T>
Image<T>::Image(size_t width, size_t height, size_t channels, const T* const data) :
	m_size({{width, height}}), m_channels(channels), m_data(new T[channels*width*height])
{
	std::copy(data, data+width*height*channels, m_data);
}

template<class T>
Image<T>::Image(const Image<T>& other) :
	m_size(other.getSize()), m_channels(other.getNumChannels()),
	m_data(new T[other.getNumChannels()*other.getWidth()*other.getHeight()])
{
	std::copy(other.m_data, other.m_data + m_size[0]*m_size[1]*m_channels, m_data);
}

template<class T>
Image<T>::Image(Image<T>&& other) :
	m_size({{0, 0}}), m_channels(0), m_data(nullptr)
{
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
			delete [] m_data;
			m_data = new T[newDataSize];
		}
		m_size = other.getSize();
		m_channels = other.getNumChannels();
		std::copy(other.m_data, other.m_data + newDataSize, m_data);
	}
	return *this;
}

template<class T>
Image<T>& Image<T>::operator=(Image<T>&& other)
{
	if (this != &other)
	{
		delete [] m_data;
		m_data = other.m_data;
		m_size = other.getSize();
		m_channels = other.getNumChannels();

		other.m_data = nullptr;
		other.m_size = {0, 0};
		other.m_channels = 0;
	}
	return *this;
}

template<class T>
Image<T>::~Image()
{
	delete [] m_data;
}

template<class T>
Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>, 0, Eigen::InnerStride<>>
Image<T>::getChannel(size_t channel)
{
	SURGSIM_ASSERT(channel < m_channels) << "channel number is larger than the number of channels";
	return Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>, 0, Eigen::InnerStride<>>(
			m_data+channel, m_size[0], m_size[1], Eigen::InnerStride<>(m_channels));
}

template<class T>
size_t Image<T>::getWidth() const
{
	return m_size[0];
}

template<class T>
size_t Image<T>::getHeight() const
{
	return m_size[1];
}

template<class T>
std::array<size_t, 2> Image<T>::getSize() const
{
	return m_size;
}

template<class T>
size_t Image<T>::getNumChannels() const
{
	return m_channels;
}

template<class T>
T* const Image<T>::getData()
{
	return m_data;
}

}
}

#endif //SURGSIM_DATASTRUCTURES_IMAGE_INL_H
