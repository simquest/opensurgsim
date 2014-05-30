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

#include <memory>
#include <utility>
#include <boost/thread.hpp>
#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{
namespace DataStructures
{

/// BufferedValue is a class to enable a representation of two values for one variable, where both values need to be
/// accessible at the same time, one in a thread safe, single threaded context, the other in a thread unsafe context.
/// \tparam T Type that is used for the value.
template <class T>
class BufferedValue
{
public:

	// Default Constructor
	BufferedValue();

	// Constructor
	/// \param value Default value.
	explicit BufferedValue(const T& value);

	/// Destructor
	~BufferedValue();

	/// Make the current value the one returned by calls to safeGet.
	void publish();

	/// Get the value
	/// \return A reference to the value.
	T& unsafeGet();

	/// Get the buffered value
	/// \return The value at the last call to publish.
	std::shared_ptr<const T> safeGet() const;

private:
	typedef boost::shared_lock<boost::shared_mutex> SharedLock;
	typedef boost::unique_lock<boost::shared_mutex> UniqueLock;

	/// The raw value
	T m_value;

	/// The buffered value
	std::shared_ptr<const T> m_safeValue;

	/// The mutex used to lock for reading and writing
	mutable boost::shared_mutex m_mutex;

};

} // DataStructures
} // SurgSim

#include "SurgSim/DataStructures/BufferedValue-inl.h"

#endif
