// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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


namespace SurgSim
{
namespace DataStructures
{

template<class T>
Image<T>::Image()
{
	ImageBase<T>::setSize(0, 0, 0);
}

template<class T>
Image<T>::Image(size_t width, size_t height, size_t channels) :
	m_data(new T[width * height * channels])
{
	ImageBase<T>::setSize(width, height, channels);
}

template<class T>
Image<T>::Image(size_t width, size_t height, size_t channels, const T* const data) :
	m_data(new T[width * height * channels])
{
	ImageBase<T>::setSize(width, height, channels);
	std::copy(data, data + width * height * channels, m_data.get());
}

template<class T>
template<class D>
Image<T>::Image(size_t width, size_t height, size_t channels, const D* const data) :
	m_data(new T[width * height * channels])
{
	ImageBase<T>::setSize(width, height, channels);
	Eigen::Map<const Eigen::Matrix<D, Eigen::Dynamic, 1>> theirData(data, width * height * channels);
	Eigen::Map<typename ImageBase<T>::VectorType, Eigen::Unaligned> myData(m_data.get(), width * height * channels);
	myData = theirData.template cast<T>();
}

template<class T>
Image<T>::Image(const Image<T>& other)
{
	ImageBase<T>::setSize(other.getWidth(), other.getHeight(), other.getNumChannels());
	size_t size = other.getWidth() * other.getHeight() * other.getNumChannels();
	m_data = std::unique_ptr<T[]>(new T[size]);
	std::copy(other.m_data.get(), other.m_data.get() + size, m_data.get());
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
		size_t oldDataSize = ImageBase<T>::getWidth() * ImageBase<T>::getHeight() * ImageBase<T>::getNumChannels();
		if (newDataSize != oldDataSize)
		{
			m_data.reset(new T[newDataSize]);
		}
		ImageBase<T>::setSize(other.getWidth(), other.getHeight(), other.getNumChannels());
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
		ImageBase<T>::setSize(other.getWidth(), other.getHeight(), other.getNumChannels());

		other.setSize(0, 0, 0);
	}
	return *this;
}

template<class T>
Image<T>::~Image()
{
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

};
};

#endif //SURGSIM_DATASTRUCTURES_IMAGE_INL_H
