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

#ifndef SURGSIM_FRAMEWORK_ACCESSIBLE_H
#define SURGSIM_FRAMEWORK_ACCESSIBLE_H

#include <string>
#include <unordered_map>
#include <functional>
#include <boost/any.hpp>

namespace SurgSim
{
namespace Framework
{

template <class T>
class Converter
{
public:
	typedef T result_type;
	T operator()(boost::any val)
	{
		return boost::any_cast<T>(val);
	}
};

template <class T>
T convert(boost::any val)
{
	return boost::any_cast<T>(val);
}

class Accessible
{
public:
	boost::any getValue(const std::string& name)
	{
		return m_getters[name]();
	}

	void setValue(const std::string& name, const boost::any& value)
	{
		m_setters[name](value);
	}

	void addGetter(const std::string& name, 
				   std::function<boost::any (void)> func)
	{
		m_getters[name] = func;
	}

	void addSetter(const std::string& name,
					std::function<void (boost::any)> func)
	{
		m_setters[name] = func;
	}



private:
	std::unordered_map<std::string, std::function<boost::any (void)> > m_getters;
	std::unordered_map<std::string, std::function<void (boost::any)> > m_setters;

};

}; // Framework
}; // SurgSim

#endif