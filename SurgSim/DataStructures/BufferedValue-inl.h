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
BufferedValue<T>::BufferedValue() :
	m_generation(0),
	m_hasWriter(false),
	m_threadId(boost::this_thread::get_id())
{
};

template <class T>
BufferedValue<T>::BufferedValue(const T& value) :
	m_values(value, value),
	m_generation(0),
	m_hasWriter(false),
	m_threadId(boost::this_thread::get_id())
{
};

template <class T>
void BufferedValue<T>::publish()
{
	UniqueLock lock(m_mutex);

	// Will eventually rollover to 0 ...
	++m_generation;

	m_values.second = m_values.first;
}

template <class T>
T* BufferedValue<T>::acquireWriteBuffer(boost::thread::id id)
{
	UniqueLock lock(m_mutex);
	SURGSIM_ASSERT(m_hasWriter == false) << "The BufferedValue only supports one writer.";
	SURGSIM_ASSERT(m_threadId == id) << "The BufferedValue can only support a writer on the same thread.";
	m_hasWriter = true;
	return &m_values.first;
}

template <class T>
void BufferedValue<T>::releaseWriteBuffer()
{
	UniqueLock lock(m_mutex);
	SURGSIM_ASSERT(m_hasWriter) << "Release called without an appropriate acquire.";
	m_hasWriter = false;
}

template <class T>
bool BufferedValue<T>::getValueIfNew(T* value, size_t* generation) const
{
	SURGSIM_ASSERT(value != nullptr) << "getValueIfNew called with nullptr for value.";
	SURGSIM_ASSERT(generation != nullptr) << "getValueIfNew called with nullptr for generation.";
	SharedLock lock(m_mutex);
	// m_generation is not monotonous as it might roll over, the incoming generation should never change i.e.
	// whenever m_generation is not the same as generation in has changes
	if (*generation != m_generation)
	{
		*value = m_values.second;
		*generation = m_generation;
		return true;
	}
	return false;
}

template <class T>
bool SurgSim::DataStructures::BufferedValue<T>::hasNewValue(size_t generation) const
{
	SharedLock lock(m_mutex);
	return generation != m_generation;
}

template <class T>
void BufferedValue<T>::getValue(T* value, size_t* generation) const
{
	SURGSIM_ASSERT(value != nullptr) << "getValue called with nullptr for value.";
	SharedLock lock(m_mutex);
	*value = m_values.second;
	if (generation != nullptr)
	{
		*generation = m_generation;
	}
}

template <class T>
const T* BufferedValue<T>::getPrivateValue() const
{
	return &m_values.first;
}

template <class T>
SafeReadAccessor<T>::SafeReadAccessor(std::shared_ptr<BufferedValue<T>> value) :
	BaseAccessor<T>(value)
{
	m_value->getValue(&m_localData, &m_generation);
}

template <class T>
const T* SafeReadAccessor<T>::updateIfNew(bool* didUpdate)
{
	SURGSIM_ASSERT(didUpdate != nullptr) << "nullptr passed.";
	*didUpdate = m_value->getValueIfNew(&m_localData, &m_generation);
	return &m_localData;
}

template <class T>
bool SafeReadAccessor<T>::isStale() const
{
	return m_value->hasNewValue(m_generation);
}

template <class T>
const T* SafeReadAccessor<T>::operator->()
{
	m_value->getValueIfNew(&m_localData, &m_generation);
	return &m_localData;
}

template <class T>
const T& SafeReadAccessor<T>::operator*()
{
	m_value->getValueIfNew(&m_localData, &m_generation);
	return m_localData;
}

template <class T>
ReadAccessor<T>::ReadAccessor(std::shared_ptr<BufferedValue<T>> value) :
	BaseAccessor<T>(value),
	m_directPointer(value->getPrivateValue())
{

}

template <class T>
const T* ReadAccessor<T>::operator->() const
{
	return m_directPointer;
}

template <class T>
const T& ReadAccessor<T>::operator*() const
{
	return *m_directPointer;
}


template <class T>
ReadWriteAccessor<T>::ReadWriteAccessor(std::shared_ptr<BufferedValue<T>> value) :
	BaseAccessor<T>(value),
	m_directPointer(value->acquireWriteBuffer(boost::this_thread::get_id()))
{

}

template <class T>
ReadWriteAccessor<T>::~ReadWriteAccessor()
{
	m_value->releaseWriteBuffer();
}

template <class T>
void ReadWriteAccessor<T>::publish()
{
	m_value->publish();
}

template <class T>
T* ReadWriteAccessor<T>::get()
{
	return m_directPointer;
}

template <class T>
T* ReadWriteAccessor<T>::operator->()
{
	return m_directPointer;
}

template <class T>
T& ReadWriteAccessor<T>::operator*()
{
	return *m_directPointer;
}

} // DataStructures
} // SurgSim

#endif
