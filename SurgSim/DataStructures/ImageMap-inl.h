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

#ifndef SURGSIM_DATASTRUCTURES_IMAGEMAP_INL_H
#define SURGSIM_DATASTRUCTURES_IMAGEMAP_INL_H


namespace SurgSim
{
namespace DataStructures
{

template<class T>
ImageMap<T>::ImageMap(size_t width, size_t height, size_t channels, T* data) :
	m_data(data)
{
	ImageBase<T>::setSize(width, height, channels);
}

template<class T>
T* const ImageMap<T>::getData()
{
	return m_data;
}

template<class T>
const T* const ImageMap<T>::getData() const
{
	return m_data;
}

};
};

#endif //SURGSIM_DATASTRUCTURES_IMAGEMAP_INL_H
