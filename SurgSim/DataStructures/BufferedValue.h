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

#ifndef SURGSIM_DATASTRUCTURES_BUFFEREDVALUE_H
#define SURGSIM_DATASTRUCTURES_BUFFEREDVALUE_H


#include <utility>
#include <boost/thread.hpp>
#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{
namespace DataStructures
{

template <class T> class ReadWriteAccessor;
template <class T> class SafeAccessor;
template <class T> class UnsafeAccessor;

/// BufferedValue is a class to enable a representation of two values for one variable, where both values need to be
/// accessible at the same time, one in a thread safe, single threaded context, the other in a thread unsafe context.
/// It is intended to be used with wrapper classes to actual perform the access.
/// Please note that all the test on for new data do not rely on the inequality of two data but on the generation count
/// if the writer calls publish() without having updated its data it will still show up as 'new' in respect to the safe
/// reader.
/// \tparam T Type that is used for the value.
template <class T>
class BufferedValue
{
public:

	// Constructor
	BufferedValue();

	/// Constructor
	/// \param value Default value.
	explicit BufferedValue(const T& value);

	typedef T Type;

	friend class ReadWriteAccessor<T>;
	friend class SafeAccessor<T>;
	friend class UnsafeAccessor<T>;

	/// Destructor
	~BufferedValue() {};

protected:

	/// Push the private value to the public side
	void publish();

	/// Get access to the internal pointer, only give access if this object was created in the same thread
	/// as the value itself.
	/// \throws SurgSim::Assertion::Failure if there is already a writer or if the thread of the writer and the buffer
	///         are not the same
	/// \param id The id of the thread of the calling function
	/// \return pointer to the internal data
	T* acquireWriteBuffer(boost::thread::id id);

	/// Release this object and enable another writer to access it
	/// \throws SurgSim::Assertion::Failure If there was no writer attached to this instance
	void releaseWriteBuffer();

	/// Copy the internal value if the internal generation count is not equal to the count passed,
	/// the internal data will be copied to the memory location passed into the function
	/// \throws SurgSim::Assertion::Failure If value or generation are nullptr
	/// \param [out] value The pointer to the memory for the new data.
	/// \param [in,out] generation The pointer to the generation count.
	/// \return true if a new value was copied, false otherwise
	bool getValueIfNew(T* value, size_t* generation) const;

	/// Determine if there is a new value available
	/// \return true if generation is not equal to the internal value
	bool hasNewValue(size_t generation) const;

	/// Copy the internal value to the memory indicated by the pointer, additionally fetch the generation count
	/// \throws SurgSim::Assertion::Failure if value is a nullptr
	/// \param [out] value The pointer to the memory for the new data.
	/// \param [out] generation The pointer to the generation count.
	void getValue(T* value, size_t* generation = nullptr) const;

	/// Expose a pointer to the private data, this is for the unfettered access.
	/// \return Address of the private data.
	const T* getPrivateValue() const;

private:

	typedef boost::shared_lock<boost::shared_mutex> SharedLock;
	typedef boost::unique_lock<boost::shared_mutex> UniqueLock;

	/// m_values.first is the thread-unsafe value, m_values.second is the thread safe side
	std::pair<T, T> m_values;

	// Indicator how often the threadsafe side has been updated, non-monotonous as it may roll over
	size_t m_generation;

	/// if true, this means that there is a writer for this value, only one writer is supported
	bool m_hasWriter;

	/// The mutex used to lock for reading and writing, shared for all the threadsafe readers
	mutable boost::shared_mutex m_mutex;

	/// indicates the thread that created this value, only writers from the same thread can access it
	boost::thread::id m_threadId;
};


/// Base class for the accessor classes, just wraps the buffer value
template <class T>
class BaseAccessor
{
public:

	BaseAccessor(std::shared_ptr<BufferedValue<T>> value) :
		m_value(value)
	{
	}

protected:
	std::shared_ptr<BufferedValue<T>> m_value;
};


/// This is the threadsafe accessor, it only provides read access via copying the data
/// use *accessor, and accessor->xxx for access
template <class T>
class SafeAccessor : public BaseAccessor<T>
{
public:

	/// Constructor
	/// \param value The value to be used.
	explicit SafeAccessor(std::shared_ptr<BufferedValue<T>> value) :
		BaseAccessor<T>(value)
	{
		m_value->getValue(&m_localData, &m_generation);
	}

	/// Explicit version of the -> operator function, update the the value if the generation counts do not match
	/// additionally return wether an update has been executed, if you need to know wether there was an update
	/// this should be faster than calling isStale() and then the update as there is only one locking operation
	/// involved.
	/// \param [out] didUpdate Address of bool for writing the result, can't be nullptr
	/// \return pointer to const data.
	const T* updateIfNew(bool* didUpdate)
	{
		SURGSIM_ASSERT(didUpdate != nullptr) << "nullptr passed.";
		*didUpdate = m_value->getValueIfNew(&m_localData, &m_generation);
		return &m_localData;
	}

	/// Check whether the data on the other side has been updated
	/// \return true if the generation counts are not equal.
	bool isStale() const
	{
		return m_value->hasNewValue(m_generation);
	}

	/// Overloaded operator for easier access
	/// \return pointer to const data.
	const T* operator->()
	{
		m_value->getValueIfNew(&m_localData, &m_generation);
		return &m_localData;
	}

	/// Overloaded operator for easier access
	/// \return reference to const data.
	const T& operator*()
	{
		m_value->getValueIfNew(&m_localData, &m_generation);
		return m_localData;
	}

private:

	using BaseAccessor<T>::m_value;

	size_t m_generation;
	T m_localData;
};

/// This is the thread unsafe accessor, it only provides read access referencing the private value, but only gives
/// const access to it,
/// use *accessor, and accessor->xxx for access
template <class T>
class UnsafeAccessor : public BaseAccessor<T>
{
public:
	UnsafeAccessor(std::shared_ptr<BufferedValue<T>> value) :
		BaseAccessor<T>(value),
		m_directPointer(value->getPrivateValue())
	{

	}

	/// Overloaded operator for easier access
	/// \return internal pointer to const data.
	const T* operator->() const
	{
		return m_directPointer;
	}

	/// Overloaded operator for easier access
	/// \return reference to const data of internal side.
	const T& operator*() const
	{
		return *m_directPointer;
	}

private:

	using BaseAccessor<T>::m_value;

	/// Direct pointer to the data, this is valid because we hold a shared pointer to the enclosing
	/// data structure, this memory is valid during this objects lifetime
	const T* m_directPointer;

};

/// Get full read and write access to the BufferedValue data structure, only one of these can be active at any given
/// time, also the write access has to be created in the same thread as the buffered value, otherwise the constructor
/// will fail.
template <class T>
class ReadWriteAccessor : public BaseAccessor<T>
{
public:

	/// Constructor
	/// \param value Pointer to BufferedValue
	explicit ReadWriteAccessor(std::shared_ptr<BufferedValue<T>> value) :
		BaseAccessor<T>(value),
		m_directPointer(value->acquireWriteBuffer(boost::this_thread::get_id()))
	{
	}

	/// Destructor
	~ReadWriteAccessor()
	{
		m_value->releaseWriteBuffer();
	}

	/// Make the internal value available to the outside
	void publish()
	{
		m_value->publish();
	}

	/// \return The actual address of the data on the private side.
	T* get()
	{
		return m_directPointer;
	}

	/// Overloaded operator for easier access.
	/// \return The actual address of the data on the private side.
	T* operator->()
	{
		return m_directPointer;
	}

	/// Overloaded operator for easier access.
	/// \return The a reference to the data on the private side.
	T& operator*()
	{
		return *m_directPointer;
	}

private:

	using BaseAccessor<T>::m_value;

	/// Direct pointer to the data, this is valid because we hold a shared pointer to the enclosing
	/// data structure, this memory is valid during this objects lifetime
	T* m_directPointer;
};

#include "SurgSim/DataStructures/BufferedValue-inl.h"

} // DataStructures
} // SurgSim


#endif
