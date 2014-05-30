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

#ifndef SURGSIM_DATASTRUCTURES_BUFFEREDVALUE_INL_H
#define SURGSIM_DATASTRUCTURES_BUFFEREDVALUE_INL_H

namespace SurgSim
{
namespace DataStructures
{

template <class T>
BufferedValue<T>::BufferedValue()
{
	m_safeValue = std::make_shared<const T>();
}

template <class T>
BufferedValue<T>::BufferedValue(const T& value) :
	m_value(value)
{
	m_safeValue = std::make_shared<const T>(m_value);
}

template <class T>
BufferedValue<T>::~BufferedValue()
{
}

template <class T>
void BufferedValue<T>::publish()
{
	UniqueLock lock(m_mutex);
	m_safeValue = std::make_shared<const T>(m_value);
}

template <class T>
T& BufferedValue<T>::unsafeGet()
{
	return m_value;
}

template <class T>
std::shared_ptr<const T> BufferedValue<T>::safeGet() const
{
	SharedLock lock(m_mutex);
	return m_safeValue;
}

} // DataStructures
} // SurgSim

#endif
