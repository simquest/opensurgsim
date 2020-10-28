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

#ifndef SURGSIM_FRAMEWORK_ACCESSIBLE_INL_H
#define SURGSIM_FRAMEWORK_ACCESSIBLE_INL_H

#include "SurgSim/Framework/Assert.h"

template <class T>
bool SurgSim::Framework::Accessible::getValue(const std::string& name, T* value) const
{
	bool result = false;
	auto functors = m_functors.find(name);
	if (value != nullptr && functors != m_functors.end() && functors->second.getter != nullptr)
	{
		try
		{
			*value = boost::any_cast<T>(functors->second.getter());
			result = true;
		}
		catch (boost::bad_any_cast&)
		{

		}
	}
	return result;
}

template <class T>
T SurgSim::Framework::Accessible::getValue(const std::string& name) const
{
	T result;
	try
	{
		result = boost::any_cast<T>(getValue(name));
	}
	catch (const boost::bad_any_cast& exception)
	{
		SURGSIM_FAILURE() << "Failure to cast to the given type. <" << exception.what() << ">";
		return T();
	}
	return result;
}

template <class T>
T SurgSim::Framework::convert(boost::any val)
{
	return boost::any_cast<T>(val);
}

#endif
