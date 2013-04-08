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

#ifndef SURGSIM_FRAMEWORK_LOCKED_CONTAINER_H
#define SURGSIM_FRAMEWORK_LOCKED_CONTAINER_H

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>


namespace SurgSim
{
namespace Framework
{

/// A simple thread-safe data container that can support multiple writers and readers.
///
/// \todo FIX THIS DOC PARAGRAPH XXX!!!XXX
/// The type of the contained data is determined by the template argument, and should satisfy the following:
///  - It must be either copy-assignable or move-assignable.  In other words, assignment must be possible using
///    either <code>operator=(const T&amp;)</code> or <code>operator=(T&amp;&amp;)</code>, or compiler-generated
///    equivalents.
///  - It must be either default-constructable or copy-constructable.  In other words, construction must be
///    possible using either <code>T()</code> or <code>T(const T&amp;)</code>, or compiler-generated equivalents.
/// Note that STL container types, plain-old-data structs, and most other things you might want to use satisfy
/// those requirements.
///
/// The container will create and manage an extra internal instance of the data object.
///
/// The interface has been designed to be incredibly simple.  The trade-off is that the overhead of reading or
/// writing to the container is significant (Each write incurs either a copy or a move of the data, plus a mutex
/// lock/unlock. Each read incurs a copy, plus a mutex lock/unlock.  Applications that write and read heavily
/// may also become mutex-bound.)
///
/// Writers write the data by calling the \ref set method, which copies or moves the data into internal storage.
/// Readers read the data by calling the \ref get method, which copies the data from internal storage.
///
/// \tparam T Generic type parameter.
template <typename T>
class LockedContainer
{
public:
	/// Create the container and the data it contains.
	///
	/// The data will be initialized using the default constructor.
	LockedContainer() :
		m_buffer(),
		m_haveNewData(false)
	{
	}

	/// Create the container and the data it contains.
	///
	/// The data will be initialized using the copy constructor.
	/// \arg initialValue The initial value to be used.
	LockedContainer(const T& initialValue) :
		m_buffer(initialValue),
		m_haveNewData(false)
	{
	}

	/// Create the container and the data it contains.
	///
	/// The data in the active buffer will be initialized using the move constructor.
	/// The data in the second, inactive buffer will be initialized using the default constructor.
	/// \arg initialValue The initial value to be moved into the active buffer.
	LockedContainer(T&& initialValue) :
		m_buffer(std::move(initialValue)),
		m_haveNewData(false)
	{
	}


	/// Destroy the container and the data it contains.
	~LockedContainer()
	{
	}

	/// Write (copy) new data into the container.
	///
	/// The data will be copied into internal storage.  If \ref set is called again before the next \ref get,
	/// the first data will be overwritten and lost.
	/// \arg value The value to be written.
	void set(const T& value)
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		m_buffer = value;
		m_haveNewData = true;
	}

	/// Write (move) new data into the container.
	///
	/// The data will be moved into internal storage.  If \ref set is called again before the next \ref get, the
	/// first data will be overwritten and lost.
	/// \arg value The value to be written.
	void set(T&& value)
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		m_buffer = std::move(value);
		m_haveNewData = true;
	}

	/// Read (copy) the data from the container.
	/// \return the data in the container, by value.
	T get() const
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		m_haveNewData = false;
		return m_buffer;
	}

	/// Read (copy) the data from the container.
	/// \param [out] value The location to write the data from the container.  The pointer must be non-null.
	void get(T* value) const
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		m_haveNewData = false;
		*value = m_buffer;
	}

	/// Read (move) the data from the container.
	/// \todo What should the LockedContainer::getMove method be called?
	/// \return the data in the container, by moving into the return value.
	T getMove()
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		m_haveNewData = false;
		return std::move(m_buffer);
	}

	/// Read (move) the data from the container.
	/// \todo What should the LockedContainer::getMove method be called?
	/// \param [out] value The location to write the data from the container.  The pointer must be non-null.
	void getMove(T* value)
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		m_haveNewData = false;
		*value = std::move(m_buffer);
	}

	/// Get the data from the container.
	/// \param [out] value The location to write the data from the container if it has changed.  The pointer
	/// 	must be non-null.
	/// \return true if there was new data (which may or may not be equal to the old).
	bool getIfChanged(T* value) const
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		if (! m_haveNewData)
		{
			return false;
		}
		else
		{
			m_haveNewData = false;
			*value = m_buffer;
			return true;
		}
	}

private:
	/// Prevent copying
	LockedContainer(const LockedContainer&);
	/// Prevent assignment
	LockedContainer& operator=(const LockedContainer&);


	/// Internal buffer
	T m_buffer;

	/// Is there data that has been written, but not yet pulled in by \ref update ?
	mutable bool m_haveNewData;

	/// Mutex for synchronization of set() and update() calls
	mutable boost::mutex m_mutex;
};

};  // namespace Framework
};  // namespace SurgSim

#endif  // SURGSIM_FRAMEWORK_LOCKED_CONTAINER_H
