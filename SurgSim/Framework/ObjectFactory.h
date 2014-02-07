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

#include <string>
#include <map>
#include <boost/function.hpp>
#include <boost/functional/factory.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

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
	template <typename Derived>
	bool registerClass(const std::string& className);

	/// Create an instance of a class based on the specific class name.
	/// \param className The class name that was used to register the class.
	/// \return a shared pointer to the object of type className, fails with an
	///         assertion otherwise.
	std::shared_ptr<Base> create(const std::string& className);

	/// Check whether the class is registered in the factory.
	/// \param className Name of the class to check.
	/// \return true if the factory has a constructor for this class
	bool isRegistered(const std::string& className) const;

private:

	typedef boost::function<std::shared_ptr<Base>()> Constructor;

	/// All the constructors.
	std::map<std::string, Constructor> m_constructors;

	/// Threadsafety for registration
	boost::mutex m_mutex;

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
	template <typename Derived>
	bool registerClass(const std::string& className);

	/// Create an instance of a class based on the specific class name.
	/// \param className The class name.
	/// \return a shared pointer to the object of type className, fails with an
	///         assertion otherwise.
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
	boost::mutex m_mutex;
};

};
};

#include "SurgSim/Framework/ObjectFactory-inl.h"

#define SURGSIM_REGISTER(BaseClass, DerivedClass) \
	static bool _surgsim_registered_##DerivedClass = BaseClass::getFactory().registerClass<DerivedClass>(#DerivedClass);


#endif // SURGSIM_FRAMEWORK_OBJECTFACTORY_H