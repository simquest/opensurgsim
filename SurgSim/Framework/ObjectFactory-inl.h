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


#ifndef SURGSIM_FRAMEWORK_OBJECTFACTORY_INL_H
#define SURGSIM_FRAMEWORK_OBJECTFACTORY_INL_H

#include "SurgSim/Framework/Assert.h"

template <class Base>
template <class Derived>
bool SurgSim::Framework::ObjectFactory<Base>::registerClass(const std::string& className)
{
	boost::mutex::scoped_lock lock(m_mutex);
	bool result = false;
	if (m_constructors.find(className) == m_constructors.end())
	{
		m_constructors[className] = boost::factory<std::shared_ptr<Derived>>();
		result = true;
	};
	return result;
};

template <class Base>
std::shared_ptr<Base> SurgSim::Framework::ObjectFactory<Base>::create(const std::string& className)
{
	boost::mutex::scoped_lock lock(m_mutex);
	auto it = m_constructors.find(className);
	if (it == m_constructors.end())
	{
		SURGSIM_FAILURE() << "ObjectFactory does not know about class called " << className;
		// gcc complains if there is no return
		return nullptr;
	}
	return (it->second)();
};


template <typename Base>
bool SurgSim::Framework::ObjectFactory<Base>::isRegistered(const std::string& className) const
{
	boost::mutex::scoped_lock lock(m_mutex);
	auto it = m_constructors.find(className);
	return (it != m_constructors.end());
}


template <typename Base, typename Parameter1>
template <typename Derived>
bool SurgSim::Framework::ObjectFactory1<Base, Parameter1>::registerClass(const std::string& className)
{
	boost::mutex::scoped_lock lock(m_mutex);
	bool result = false;
	if (m_constructors.find(className) == m_constructors.end())
	{
		m_constructors[className] = boost::factory<std::shared_ptr<Derived>>();
		result = true;
	};
	return result;
};

template <typename Base, typename Parameter1>
std::shared_ptr<Base> SurgSim::Framework::ObjectFactory1<Base, Parameter1>::create(
	const std::string& className,
	const Parameter1& val)
{
	boost::mutex::scoped_lock lock(m_mutex);
	auto it = m_constructors.find(className);

	if (it == m_constructors.end())
	{
		SURGSIM_FAILURE() << "ObjectFactory does not know about class called " << className;
		// gcc complains if there is no return
		return nullptr;
	}
	return (it->second)(val);
};

template <typename Base, typename Parameter1>
bool SurgSim::Framework::ObjectFactory1<Base, Parameter1>::isRegistered(const std::string& className) const
{
	boost::mutex::scoped_lock lock(m_mutex);
	auto it = m_constructors.find(className);
	return (it != m_constructors.end());
}


#endif // SURGSIM_FRAMEWORK_OBJECTFACTORY_INL_H