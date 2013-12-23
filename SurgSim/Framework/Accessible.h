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
#include <memory>
#include <unordered_map>
#include <functional>
#include <boost/any.hpp>
#include <yaml-cpp/yaml.h>

#include "SurgSim/Math/Matrix.h"

#include "SurgSim/Serialize/Convert.h"

namespace SurgSim
{
namespace Framework
{

/// Mixin class for enabling a property system on OSS classes, the instance still needs to initialise properties in
/// the constructor by using either addSetter, addGetter, addAccessors or the macro for each member variable
/// that should be made accessible.
class Accessible
{
public:

	typedef std::function<boost::any(void)> GetterType;
	typedef std::function<void (boost::any)> SetterType;

	typedef std::function<YAML::Node(void)> EncoderType;
	typedef std::function<void(const YAML::Node*)> DecoderType;

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
	/// \param name Name of the property to be checked.
	/// \return true if the property exists and has a getter
	bool isReadable(const std::string& name) const;

	/// Check whether a property is writeable
	/// \param name Name of the property to be checked.
	/// \return true if the property exists and has a setter
	bool isWriteable(const std::string& name) const;

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

	/// Sets the functions used to convert data from and to a YAML::Node. Will throw and exception
	/// if the data type that is passed to YAML cannot be converted into a YAML::Node
	/// \param name The name of the property.
	/// \param encoder The function to be used to put the property into the node.
	/// \param decoder The function to be used to read the property from the node and set it
	///                in the instance.
	void setSerializable(const std::string& name, EncoderType encoder, DecoderType decoder);

	/// Encode this Accessible to a YAML::Node
	/// \return The encoded version of this instance.
	YAML::Node encode();

	/// Decode this Accessible from a YAML::Node, will throw an exception if the data type cannot
	/// be converted.
	/// \param node The node that carries the data to be, properties with names that don't
	///             match up with properties in the Accessible are ignored
	void decode(const YAML::Node& node);

private:

	/// Private struct to keep the map under control
	struct Functors
	{
		GetterType getter;
		SetterType setter;
		EncoderType encoder;
		DecoderType decoder;
	};

	std::unordered_map<std::string, Functors> m_functors;

};

/// Public struct to pair an accessible with its appropriate property
struct Property
{
	std::weak_ptr<Accessible> accessible;
	std::string name;
};

/// Wrap boost::any_cast to use in std::bind, for some reason it does not work by itself. This function will
/// throw an exception if the cast does not work, this usually means that the types do not match up at all.
/// \tparam T target type for conversion.
/// \param val The value to be converted.
/// \return An object converted from boost::any to T, will throw an exception if the conversion fails
template <class T>
T convert(boost::any val);

/// Specialization for convert<T>() to correctly cast Matrix44d to Matrix44f, will throw if the val is not casteable to
/// Matrix44[fd]. This is necessary as we need Matrix44f as outputs in some cases but all our Matrices are Matrix44d.
/// This lets the user define a property that does a type conversion, without having to implement an accessor.
/// \param val The value to be converted, should be a Matrix44[df].
/// \return A matrix val converted to Matrix44f.
template <>
SurgSim::Math::Matrix44f convert(boost::any val);

/// A macro to register getter and setter for a property that is readable and writeable,
/// order of getter and setter agrees with 'RW'. Note that the property should not be quoted in the original
/// macro call.
#define SURGSIM_ADD_RW_PROPERTY(class, type, property, getter, setter) \
	setAccessors(#property, \
				std::bind(&class::getter, this),\
				std::bind(&class::setter, this, std::bind(SurgSim::Framework::convert<type>,std::placeholders::_1)))

#define SURGSIM_ADD_SERIALIZABLE_PROPERTY(class, type, property, getter, setter) \
	setAccessors(#property, \
				std::bind(&class::getter, this),\
				std::bind(&class::setter, this, std::bind(SurgSim::Framework::convert<type>,std::placeholders::_1)));\
	setSerializable(#property,\
				std::bind(&YAML::convert<type>::encode, std::bind(&class::getter, this)),\
				std::bind(&class::setter, this, std::bind(&YAML::Node::as<type>,std::placeholders::_1)))

/// A macro to register a getter for a property that is read only
#define SURGSIM_ADD_RO_PROPERTY(class, type, property, getter) \
	setGetter(#property, \
	std::bind(&class::getter, this))


}; // Framework
}; // SurgSim

#include "SurgSim/Framework/Accessible-inl.h"

#endif