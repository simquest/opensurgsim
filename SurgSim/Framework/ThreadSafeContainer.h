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

#ifndef SURGSIM_FRAMEWORK_THREAD_SAFE_CONTAINER_H
#define SURGSIM_FRAMEWORK_THREAD_SAFE_CONTAINER_H

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>


namespace SurgSim
{
namespace Framework
{

/// A thread-safe data container that supports multiple writers and a single reader.
///
/// The type of the contained data is determined by the template argument, and should satisfy the following:
///  - It must be either copy-assignable or move-assignable.  In other words, assignment must be possible using
///    either <code>operator=(const T&)</code> or <code>operator=(T&&)</code>, or compiler-generated equivalents.
///  - It must be either default-constructable or copy-constructable.  In other words, construction must be
///    possible using either <code>T()</code> or <code>T(const T&)</code>, or compiler-generated equivalents.
/// Note that STL container types, plain-old-data structs, and most other things you might want to use satisfy
/// those requirements.
///
/// The container will create two instances of the data internally.
///
/// Writers set the data by calling the \ref set method, which copies or moves the data into internal storage.
///
/// The reader requests its view of the data to be updated by calling the \ref update method, and can access the data
/// between updates by "dereferencing" the container using the <code>*</code> or <code>-&gt;</code> operators.
template <typename T>
class ThreadSafeContainer
{
public:
	/// Create the container and the data it contains.
	///
	/// The data will be initialized using the default constructor.
	ThreadSafeContainer() :
		m_buffer0(),
		m_buffer1(),
		m_readBuffer(&m_buffer0),
		m_writeBuffer(&m_buffer1),
		m_haveNewData(false)
	{
	}

	/// Create the container and the data it contains.
	///
	/// The data will be initialized using the copy constructor.
	/// \arg initialValue The initial value to be used.
	ThreadSafeContainer(const T& initialValue) :
		m_buffer0(initialValue),
		m_buffer1(initialValue),
		m_readBuffer(&m_buffer0),
		m_writeBuffer(&m_buffer1),
		m_haveNewData(false)
	{
	}

	/// Create the container and the data it contains.
	///
	/// The data in the active buffer will be initialized using the move constructor.
	/// The data in the second, inactive buffer will be initialized using the default constructor.
	/// \arg initialValue The initial value to be moved into the active buffer.
	ThreadSafeContainer(T&& initialValue) :
		m_buffer0(std::move(initialValue)),
		m_buffer1(),
		m_readBuffer(&m_buffer0),
		m_writeBuffer(&m_buffer1),
		m_haveNewData(false)
	{
	}


	/// Destroy the container and the data it contains.
	~ThreadSafeContainer()
	{
		m_readBuffer = m_writeBuffer = 0;
	};

	/// Write (copy) new data into the container.
	///
	/// The data will be copied into internal storage, but will not be seen by the reader until it calls
	/// \ref update.  If \ref set is called again before \ref update, the first data will be overwritten.
	/// \arg value The value to be written.
	void set(const T& value)
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		*m_writeBuffer = value;
		m_haveNewData = true;
	}

	/// Write (move) new data into the container.
	///
	/// The data will be moved into internal storage, but will not be seen by the reader until it calls
	/// \ref update.  If \ref set is called again before \ref update, the first data will be overwritten.
	/// \arg value The value to be written.
	void set(T&& value)
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		*m_writeBuffer = std::move(value);
		m_haveNewData = true;
	}

	/// Get the currently active data for reading.
	///
	/// The data is not copied, so the value becomes invalid after the next \ref update call.
	/// \return a (non-modifiable) pointer to the active data.
	const T* get() const
	{
		// Locking is not needed here (on x86/x64, and assuming get() runs in the same thread as update()).
		return m_readBuffer;
	}

	/// Access the currently active data for reading.
	///
	/// The data is not copied, so the value becomes invalid after the next \ref update call.
	/// \return a (non-modifiable) pointer to the active data.
	const T* operator->() const
	{
		return get();
	}

	/// Access the currently active data for reading.
	///
	/// The data is not copied, so the value becomes invalid after the next \ref update call.
	/// \return a (non-modifiable) reference to the active data.
	const T& operator*() const
	{
		return *(get());
	}

	/// Update the output value from the last set value.
	/// The output value is guaranteed not to change \b except when this call is made.
	/// \return true if there was new data (which may or may not be equal to the old).
	bool update()
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		if (m_haveNewData)
		{
			std::swap(m_readBuffer, m_writeBuffer);
			// reads will now use the written buffer; next write goes to what as
			m_haveNewData = false;
			return true;
		}
		else
		{
			return false;
		}
	}

private:
	/// Prevent copying
	ThreadSafeContainer(const ThreadSafeContainer&);
	/// Prevent assignment
	ThreadSafeContainer& operator=(const ThreadSafeContainer&);


	/// First internal buffer
	T m_buffer0;

	/// Second internal buffer
	T m_buffer1;

	/// Pointer to the buffer currently active for reading.
	/// Access to the data is always read-only, but we don't declare it \c const
	/// so we can swap it with the write buffer.
	T* m_readBuffer;

	/// Pointer to the buffer currently active for writing.
	T* m_writeBuffer;

	/// Is there data that has been written, but not yet pulled in by \ref update ?
	bool m_haveNewData;

	/// Mutex for synchronization of set() and update() calls
	boost::mutex m_mutex;
};

};  // namespace Framework
};  // namespace SurgSim

#endif  // SURGSIM_FRAMEWORK_THREAD_SAFE_CONTAINER_H
