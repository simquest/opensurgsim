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
T convert(boost::any val)
{
	return boost::any_cast<T>(val);
}

/// Mixin class for enabling a property system on OSS classes, the instance still needs to initialise properties in
/// the constructor by using either addSetter, addGetter, addAccessors or the macro for each member variable
/// that should be made accessible.
class Accessible
{
public:

	typedef std::function<boost::any(void)> GetterType;
	typedef std::function<void (boost::any)> SetterType;

	/// Retrieves the value with the name by executing the getter if it is found.
	/// \param	name	The name of the property.
	/// \return	The value of the property if the getter was found, a default constructed boost::any
	/// 		if it was not found.
	boost::any getValue(const std::string& name);

	/// Retrieves the value with the name by executing the getter if it is found, and converts it to
	/// the type of the output parameter.
	/// \tparam T	the type of the property, usually can be deduced automatically
	/// \param	name	The name of the property.
	/// \param [out]	value	If non-null, will receive the value of the given property.
	/// \return	true if value != nullptr and the getter can be found.
	template <class T>
	bool getValue(const std::string& name, T* value);

	/// Sets a value of a property that has setter.
	/// \param	name 	The name of the property.
	/// \param	value	The value that it should be set to.
	void setValue(const std::string& name, const boost::any& value);

	/// Check whether a property is readable
	/// \return true if the property exists and has a getter
	bool isReadable(const std::string& name);

	/// Check whether a property is writeable
	/// \return true if the property exists and has a setter
	bool isWriteable(const std::string& name);

	/// Sets a getter for a given property.
	/// \param	name	The name of the property.
	/// \param	func	The getter function.
	void setGetter(const std::string& name, GetterType func);

	/// Sets a setter for a given property.
	/// \param	name	The name of the property.
	/// \param	func	The setter function.
	void setSetter(const std::string& name, SetterType func);

	/// Sets the accessors getter and setter in one function.
	/// \param	name  	The name of the property.
	/// \param	getter	The getter.
	/// \param	setter	The setter.
	void setAccessors(const std::string& name, GetterType getter, SetterType setter);


private:

	std::unordered_map<std::string, GetterType > m_getters;
	std::unordered_map<std::string, SetterType > m_setters;
};

#define SURGSIM_ADD_RW_PROPERTY(class, type, property, getter, setter) \
	setAccessors(#property, \
				std::bind(&class::getter, this),\
				std::bind(&class::setter, this, std::bind(SurgSim::Framework::convert<type>,std::placeholders::_1)))

}; // Framework
}; // SurgSim

#include "SurgSim/Framework/Accessible-inl.h"

#endif