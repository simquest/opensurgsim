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

namespace SurgSim
{
namespace Framework
{

/// A factory implementation for Objects
template <typename Base>
class ObjectFactory
{
public:

	/// Template version to register a shape into the internal directory.
	/// \tparam T The specific type of the shape to be registered.
	/// \param className The identifier name to be used.
	template <typename Derived>
	void registerClass(const std::string& className);

	/// Create an instance of derived rigid shape based on the specific class name.
	/// \param className The identifier name to be used.
	/// \return a pointer to the object of derived rigid shape, or nullptr otherwise.
	std::shared_ptr<Base> create(const std::string& className);

private:
	/// A wrapper of function object
	typedef boost::function<std::shared_ptr<Base>()> Constructor;

	/// Look up table for shapes factory.
	std::map<std::string, Constructor> m_constructors;

};

/// A factory implementation for Objects
template <typename Base, typename Parameter1>
class ObjectFactory1
{
public:

	/// Template version to register a shape into the internal directory.
	/// \tparam T The specific type of the shape to be registered.
	/// \param className The identifier name to be used.
	template <typename Derived>
	void registerClass(const std::string& className);

	/// Create an instance of derived rigid shape based on the specific class name.
	/// \param className The identifier name to be used.
	/// \return a pointer to the object of derived rigid shape, or nullptr otherwise.
	std::shared_ptr<Base> create(const std::string& className, const Parameter1& val);

private:
	/// A wrapper of function object
	typedef boost::function<std::shared_ptr<Base>(Parameter1)> Constructor;

	/// Look up table for shapes factory.
	std::map<std::string, Constructor> m_constructors;

};

};
}; 

#include "SurgSim/Framework/ObjectFactory-inl.h"

#endif // SURGSIM_SERIALIZE_SHAPESFACTORY_H