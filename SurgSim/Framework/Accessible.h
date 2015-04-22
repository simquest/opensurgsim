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

namespace SurgSim
{
namespace Framework
{

/// Mixin class for enabling a property system on OSS classes, the instance still needs to initialize properties in
/// the constructor by using either addSetter, addGetter, addAccessors or the macro for each member variable
/// that should be made accessible.
class Accessible
{
public:

	/// Default Constructor
	Accessible();

	/// Destructor
	~Accessible();

	typedef std::function<boost::any(void)> GetterType;
	typedef std::function<void (boost::any)> SetterType;

	typedef std::function<YAML::Node(void)> EncoderType;
	typedef std::function<void(const YAML::Node*)> DecoderType;



	/// Retrieves the value with the name by executing the getter if it is found and tries to convert
	/// it to the given type.
	/// \throws SurgSim::Framework::AssertionFailure If the conversion fails or the property cannot be found.
	/// \tparam T The requested type for the property.
	/// \param	name	The name of the property.
	/// \return	The value of the property if the getter was found
	template <class T>
	T getValue(const std::string& name) const;

	/// Retrieves the value with the name by executing the getter if it is found.
	/// \throws SurgSim::Framework::AssertionFailure if the property cannot be found
	/// \param	name	The name of the property.
	/// \return	The value of the property if the getter was found
	boost::any getValue(const std::string& name) const;


	/// Retrieves the value with the name by executing the getter if it is found, and converts it to
	/// the type of the output parameter. This does not throw.
	/// \tparam T	the type of the property, usually can be deduced automatically
	/// \param	name	The name of the property.
	/// \param [out]	value	If non-null, will receive the value of the given property.
	/// \return	true if value != nullptr and the getter can be found.
	template <class T>
	bool getValue(const std::string& name, T* value) const;

	/// Sets a value of a property that has setter.
	/// \throws SurgSim::Framework::AssertionFailure If the property cannot be found.
	/// \param	name 	The name of the property.
	/// \param	value	The value that it should be set to.
	void setValue(const std::string& name, const boost::any& value);

	/// Check whether a property is readable
	/// \param name Name of the property to be checked.
	/// \return true if the property exists and has a getter
	bool isReadable(const std::string& name) const;

	/// Check whether a property is writable
	/// \param name Name of the property to be checked.
	/// \return true if the property exists and has a setter
	bool isWriteable(const std::string& name) const;

	/// Sets a getter for a given property.
	/// \throws SurgSim::Framework::AssertionFailure if func is a nullptr.
	/// \param	name	The name of the property.
	/// \param	func	The getter function.
	void setGetter(const std::string& name, GetterType func);

	/// Sets a setter for a given property.
	/// \throws SurgSim::Framework::AssertionFailure if func is a nullptr.
	/// \param	name	The name of the property.
	/// \param	func	The setter function.
	void setSetter(const std::string& name, SetterType func);

	/// Sets the accessors getter and setter in one function.
	/// \throws SurgSim::Framework::AssertionFailure if either getter or setter is a nullptr.
	/// \param	name  	The name of the property.
	/// \param	getter	The getter.
	/// \param	setter	The setter.
	void setAccessors(const std::string& name, GetterType getter, SetterType setter);

	/// Removes all the accessors (getter and setter) for a given property
	/// \param name The name of the property
	void removeAccessors(const std::string& name);

	/// Adds a property with the given name that uses the targets accessors, in effect forwarding the value
	/// to the target
	/// \note This will copy the appropriate calls into the local function table of this accessible, in effect
	///       exposing a pointer to the target, if the target goes out of scope, the behavior is undefined
	/// \throws SurgSim::Framework::AssertionFailure if the target does not contain the property named in this call.
	/// \param name The name of the new property
	/// \param target The instance that provides the actual property
	/// \param targetProperty The name of the property that should be used.
	void forwardProperty(const std::string& name, const Accessible& target, const std::string& targetProperty);

	/// Sets the functions used to convert data from and to a YAML::Node. Will throw an exception
	/// if the data type that is passed to YAML cannot be converted into a YAML::Node
	/// \param name The name of the property.
	/// \param encoder The function to be used to put the property into the node.
	/// \param decoder The function to be used to read the property from the node and set it
	///                in the instance.
	void setSerializable(const std::string& name, EncoderType encoder, DecoderType decoder);

	/// Sets the functions used to convert data from a YAML::Node.
	/// This leaves the encoder (class -> YAML) conversion empty, this can be used to let the user decide how to
	/// model the data in the data file, inside the class this should all result in one member to be created/changed.
	/// \param name The name of the property.
	/// \param decoder The function to be used to read the property from the node and set it
	///                in the instance.
	void setDecoder(const std::string& name, DecoderType decoder);


	/// Encode this Accessible to a YAML::Node
	/// \return The encoded version of this instance.
	YAML::Node encode() const;

	/// Decode this Accessible from a YAML::Node, will throw an exception if the data type cannot
	/// be converted.
	/// \throws SurgSim::Framework::AssertionFailure if node is not of YAML::NodeType::Map.
	/// \param node The node that carries the data to be decoded, properties with names that don't
	///             match up with properties in the Accessible will be reported.
	/// \param ignoredProperties Properties that will be ignored.
	void decode(const YAML::Node& node, const std::vector<std::string>& ignoredProperties = std::vector<std::string>());

private:

	/// @{
	/// Prevent default copy construction and default assignment
	Accessible(const Accessible& other) /*= delete*/;
	Accessible& operator=(const Accessible& other) /*= delete*/;
	/// @}

	/// Private struct to keep the map under control
	struct Functors
	{
		Functors() : getter(nullptr), setter(nullptr), encoder(nullptr), decoder(nullptr) {}
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

template <>
boost::any Accessible::getValue(const std::string& name) const;


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

/// A macro to register a getter for a property that is read only
#define SURGSIM_ADD_RO_PROPERTY(class, type, property, getter) \
	setGetter(#property, \
	std::bind(&class::getter, this))

/// A macro to register a serializable property, this needs to support reading, writing and all the
/// conversions to and from YAML::Node
#define SURGSIM_ADD_SERIALIZABLE_PROPERTY(class, type, property, getter, setter) \
	setAccessors(#property, \
				std::bind(&class::getter, this),\
				std::bind(&class::setter, this, std::bind(SurgSim::Framework::convert<type>,std::placeholders::_1)));\
	setSerializable(#property,\
				std::bind(&YAML::convert<type>::encode, std::bind(&class::getter, this)),\
				std::bind(&class::setter, this, std::bind(&YAML::Node::as<type>,std::placeholders::_1)))

/// A macro to register a setter that can be used from YAML, and as a writeable property
/// use this to provide alternatives to more complicated values, e.g. setModelFilename() vs generate and set Model
// Enables the alternative use of the model file instead of the actual mesh object
#define SURGSIM_ADD_SETTER(class, type, property, setter) \
	{\
		setDecoder(#property, std::bind((void(class::*)(type))&class::setter, this,\
					std::bind(&YAML::Node::as<type>,std::placeholders::_1))); \
		setSetter(#property, std::bind((void(class::*)(type))&class::setter, this,\
					std::bind(SurgSim::Framework::convert<type>,std::placeholders::_1)));\
	}
}; // Framework
}; // SurgSim

#include "SurgSim/Framework/Accessible-inl.h"

#endif
