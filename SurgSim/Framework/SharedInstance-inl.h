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

#ifndef SURGSIM_FRAMEWORK_SHAREDINSTANCE_INL_H
#define SURGSIM_FRAMEWORK_SHAREDINSTANCE_INL_H

#include <memory>
#include <functional>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include "SurgSim/Framework/Assert.h"


namespace SurgSim
{
namespace Framework
{

template <typename T>
SharedInstance<T>::SharedInstance() :
	m_instanceCreator(defaultInstanceCreator())
{
}

template <typename T>
SharedInstance<T>::SharedInstance(const InstanceCreator& instanceCreator) :
	m_instanceCreator(instanceCreator)
{
}

template <typename T>
SharedInstance<T>::~SharedInstance()
{
}

template <typename T>
std::shared_ptr<T> SharedInstance<T>::get()
{
	boost::lock_guard<boost::mutex> lock(m_mutex);
	std::shared_ptr<T> instance = m_weakInstance.lock();
	if (! instance)
	{
		instance = createInstance();
		m_weakInstance = instance;
	}
	return std::move(instance);
}

template <typename T>
std::shared_ptr<T> SharedInstance<T>::createInstance()
{
	std::shared_ptr<T> instance = m_instanceCreator();
	SURGSIM_ASSERT(instance);
	return std::move(instance);
}

template <typename T>
typename SharedInstance<T>::InstanceCreator SharedInstance<T>::defaultInstanceCreator()
{
	return []() { return std::make_shared<T>(); };  // NOLINT(readability/braces)
}


};  // namespace Framework
};  // namespace SurgSim

#endif  // SURGSIM_FRAMEWORK_SHAREDINSTANCE_INL_H
