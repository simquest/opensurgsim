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

#ifndef SURGSIM_FRAMEWORK_OBJECTFACTORY_H
#define SURGSIM_FRAMEWORK_OBJECTFACTORY_H

#include "SurgSim/Framework/Macros.h"

#include <string>
#include <map>
#include <boost/function.hpp>
#include <boost/functional/factory.hpp>
#include <boost/thread/mutex.hpp>


namespace SurgSim
{
namespace Framework
{

/// An object factory, once a class is registered with the factory it can
/// be used to create instances of registered classes. All the classes registered
/// need to have a default constructor.
/// \note The names used for registration and the actual c++ class names are independent
///       the className parameter here is just an identifier for the class.
/// \tparam Base the base class for all classes that can be registered with the factory.
template <typename Base>
class ObjectFactory
{
public:

	/// Register a class with the factory.
	/// \tparam T The specific type of the class to be registered.
	/// \param className The name of this class.
	/// \return true if the class was added, false if it already existed in the registry.
	template <typename Derived>
	bool registerClass(const std::string& className);

	/// Create an instance of a class based on the specific class name.
	/// \param className The class name that was used to register the class.
	/// \return a shared pointer to the object of type className, fails with an
	///         assertion otherwise.
	std::shared_ptr<Base> create(const std::string& className);

	/// Check whether the class is registered in the factory.
	/// \param className Name of the class to check.
	/// \return true if the factory has a constructor for this class.
	bool isRegistered(const std::string& className) const;

private:

	typedef boost::function<std::shared_ptr<Base>()> Constructor;

	/// All the constructors.
	std::map<std::string, Constructor> m_constructors;

	/// Threadsafety for registration
	mutable boost::mutex m_mutex;

};

/// An object factory, once a class is registered with the factory it can
/// be used to create instances of registered classes. All the classes registered
/// need to have a one parameter constructor, the type for that parameter can be
/// passed as a template parameter.
/// \note The names used for registration and the actual c++ class names are independent
///       the className parameter here is just an identifier for the class.
/// \tparam Base The base class for all classes that can be registered with the factory.
/// \tparam Parameter1 The class for the constructor parameter.
template <typename Base, typename Parameter1>
class ObjectFactory1
{
public:

	/// Register a class with the factory.
	/// \tparam T The specific type of the class to be registered.
	/// \param className The name of this class.
	/// \return true if the class was added, false if it already existed in the registry.
	template <typename Derived>
	bool registerClass(const std::string& className);

	/// Create an instance of a class based on the specific class name, whose constructor takes 1 parameter.
	/// \param className The class name.
	/// \param val The value of the parameter.
	/// \return a shared pointer to the object of type className instantiated with the given parameter,
	///         fails with an assertion otherwise.
	std::shared_ptr<Base> create(const std::string& className, const Parameter1& val);

	/// Check whether the class is registered in the factory.
	/// \param className Name of the class to check.
	/// \return true if the factory has a constructor for this class
	bool isRegistered(const std::string& className) const;

private:

	typedef boost::function<std::shared_ptr<Base>(Parameter1)> Constructor;

	/// All the constructors.
	std::map<std::string, Constructor> m_constructors;

	/// Threadsafety for registration
	mutable boost::mutex m_mutex;
};


/// CRTP Base class to implement Object Factory functionality on a base class, use this rather than writing
/// your own functions to return the factory
/// \tparam T base class of the generated objects
template <class T>
class FactoryBase
{
public:
	typedef ObjectFactory<T> FactoryType;

	/// \return a reference to the factory
	static FactoryType& getFactory()
	{
		static FactoryType factory;
		return factory;
	}
};

/// CRTP Base class to implement Object Factory functionality on a base class, use this rather than writing
/// your own functions to return the factory
/// \tparam T base class of the generated objects
/// \tparam P constructor parameter for object generation
template <class T, class P>
class FactoryBase1
{
public:
	typedef ObjectFactory1<T, P> FactoryType;

	/// \return a reference to the factory
	static FactoryType& getFactory()
	{
		static FactoryType factory;
		return factory;
	}
};

};
};

#include "SurgSim/Framework/ObjectFactory-inl.h"

/// Register a class with a factory that is in a base class, DerivedClass has to be of type BaseClass.
/// The assignment is used to enable the execution of registerClass during static initialization time.
/// This macro should be put under the same namespace in which 'ClassName' is, to avoid symbol clashes.
/// 'BaseClass' should be the full name of the base class with namespace prefix,
/// 'DerivedClass' is 'ClassName' with namespace prefixes,
/// and 'ClassName' is the name of the class without namespace prefix.
#define SURGSIM_REGISTER(BaseClass, DerivedClass, ClassName) \
	bool SURGSIM_CONCATENATE(ClassName, Registered) = \
		BaseClass::getFactory().registerClass<DerivedClass>(#DerivedClass); \

/// Force compilation of the boolean symbol SURGSIM_CONCATENATE(ClassName, Registered) in SURGSIM_REGISTER macro,
/// which in turn registers DerivedClass into BaseClass's ObjectFactory.
/// After that, DerivedClass is linked to any code which includes its header.
///
/// Boolean symbol SURGSIM_CONCATENATE(ClassName, Registered) in SURGSIM_REGISTER macro is exposed as an
/// extern variable in DerivedClass's header, and is referenced to initialize the static global variable
/// SURGSIM_CONCATENATE(ClassName, IsRegistered) in the header.
///
/// This forces the compiler to include the definition of SURGSIM_CONCATENATE(ClassName, Registered)
/// (defined most likely in the cpp file).
/// The variable SURGSIM_CONCATENATE(ClassName, IsRegistered) will never be used, so the GCC warning is disabled.
/// This macro should be put in the DerivedClass's header file, under the same namespace in which the DerivedClass is.
/// 'ClassName' should be the name of the class without any prefix.
#define SURGSIM_STATIC_REGISTRATION(ClassName) \
	SURGSIM_DO_PRAGMA (GCC diagnostic push); \
	SURGSIM_DO_PRAGMA (GCC diagnostic ignored "-Wunused-variable"); \
	extern bool SURGSIM_CONCATENATE(ClassName, Registered); \
	static bool SURGSIM_CONCATENATE(ClassName, IsRegistered) = SURGSIM_CONCATENATE(ClassName, Registered); \
	SURGSIM_DO_PRAGMA (GCC diagnostic pop)

#endif // SURGSIM_FRAMEWORK_OBJECTFACTORY_H