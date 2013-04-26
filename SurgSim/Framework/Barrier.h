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

// Based on barrier.hpp from Boost 1.51
// Copyright (C) 2002-2003
// David Moore, William E. Kempf
// Copyright (C) 2007-8 Anthony Williams
//
// Which was distributed under the Boost Software License, Version 1.0.
// (See accomanying NOTICES or a copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef SURGSIM_FRAMEWORK_BARRIER_H
#define SURGSIM_FRAMEWORK_BARRIER_H

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <string>
#include <stdexcept>

#include <SurgSim/Framework/Assert.h>

namespace SurgSim
{
namespace Framework
{

/// Barrier class, synchronize a set of threads to wait at a common point, all
/// threads will wait at Barrier::wait(val) until the number of threads calling
/// wait is equal to the number given in the constructor.
/// Additionally wait will return a boolean AND over all the values passed into
/// the wait function, this can be used to signal a failure condition across
/// threads.
class Barrier
{
public:
	/// Construct the barrier.
	/// \param count Number of threads to synchronize, can't be 0.
	explicit Barrier(unsigned int count);

	/// Waits until all \a count threads have called wait.
	///
	/// The wait calls in all of the threads waiting on a barrier will return
	/// the same value.  This return value will be true if the \c success
	/// argument was true in \em all of the threads; if any thread passes false,
	/// the return value will be false.
	/// \param success a value indicating if this thread has been successful, used to determine the return
	/// 	value across all threads.
	/// \return true if all threads claimed success, false otherwise.
	bool wait(bool success);

private:
	boost::mutex m_mutex;
	boost::condition_variable m_cond;
	unsigned int m_threshold;
	unsigned int m_count;
	unsigned int m_generation;
	bool m_success;
	bool m_successResult;
};

} // namespace Framework
} // namespace SurgSim

#endif
