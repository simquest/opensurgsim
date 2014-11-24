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

template <class T, class ST>
BufferedValue<T, ST>::BufferedValue()
{
	m_safeValue = std::make_shared<const ST>();
}

template <class T, class ST>
BufferedValue<T, ST>::BufferedValue(const T& value) :
	m_value(value)
{
	publish();
}

template <class T, class ST>
BufferedValue<T, ST>::~BufferedValue()
{
}

template <class T, class ST>
void BufferedValue<T, ST>::publish()
{
	doPublish<T>(0);
}

template <class T, class ST>
T& BufferedValue<T, ST>::unsafeGet()
{
	return m_value;
}

template <class T, class ST>
std::shared_ptr<const ST> BufferedValue<T, ST>::safeGet() const
{
	SharedLock lock(m_mutex);
	return m_safeValue;
}

template <class T, class ST>
template <typename V>
void BufferedValue<T, ST>::doPublish(decltype(typename V::const_iterator(), int()))
{
	UniqueLock lock(m_mutex);
	m_safeValue = std::make_shared<const ST>(m_value.cbegin(), m_value.cend());
}

template <class T, class ST>
template <typename>
void BufferedValue<T, ST>::doPublish(...)
{
	UniqueLock lock(m_mutex);
	m_safeValue = std::make_shared<const ST>(m_value);
};


} // DataStructures
} // SurgSim

#endif
