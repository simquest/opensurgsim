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

#ifndef SURGSIM_FRAMEWORK_REUSEFACTORY_H
#define SURGSIM_FRAMEWORK_REUSEFACTORY_H

#include <memory>
#include <stack>

namespace SurgSim
{

namespace Framework
{

/// Factory for acquiring new or unused existing instances of class T to reduce repeated deallocation and reallocation
/// of objects with short lifespans.
///
/// Example Usage:
/// \code{.cpp}
/// {
///		// Create an instance of the factory to provide instances of MyObject.
///		ReuseFactory<MyObject> factory;
///		// Get an instance of MyObject. This instance will be allocated as no unused objects are available yet.
///		std::shared_ptr<MyObject> myObject = factory.getInstance();
///		// Setup the provided object, as the state is not guaranteed to be initialized.
///		myObject.set(...);
/// }
/// // When myObject goes out of scope, it will automatically be added back to the factory for reuse later.
/// // This will return the previous, now unused, instance rather than allocating anew.
/// std::shared_ptr<MyObject> myObject2 = factory.getInstance();
/// // Setup the provided object, as the state is not guaranteed to be initialized.
/// myObject2.set(...);
/// \endcode
///
/// Limitations:
/// - The class T must have a default constructor.
/// - The state of returned objects are in no way reset, so the state will need to be setup after retrieving the
///   instance from the factory.
/// - Custom deleters for an instance of T cannot be specified, as a custom deleter is used to manage the unused
///   objects.
///
/// \tparam T	Instances of this class are provided by this factory
template <class T>
class ReuseFactory
{
	/// Custom Deleter is friended to manage unused objects rather than actually deleting them.
	friend class Deleter;
public:
	/// Constructor. Initially no unused objects are available, so returned instances are new allocations.
	ReuseFactory() : deleter(this) {}
	/// Destructor. Any remaining unused objects will be deleted.
	~ReuseFactory() {}

	/// Get a new or previously deleted object of class T.
	/// \return Valid shared pointer to an object of class T
	std::shared_ptr<T> getInstance()
	{
		std::shared_ptr<T> object;

		if (m_unusedObjects.empty())
		{
			object = std::shared_ptr<T>(new T(), deleter);
		}
		else
		{
			object = std::shared_ptr<T>(m_unusedObjects.top().release(), deleter);
			m_unusedObjects.pop();
		}

		return object;
	}

private:
	/// Custom deleter to keep unused objects for reuse, rather than actually deleting them.
	class Deleter
	{
	public:
		/// Constructor
		/// \param factory ReuseFactory with the collection of unused object for reuse.

		explicit Deleter(ReuseFactory* factory) : m_factory(factory)
		{
		}
		/// Deletion method, adds the object to the ReuseFactory's collection.
		/// \param unusedObject Object that is no longer referenced by any shared pointers
		void operator()(T* unusedObject) const
		{
			m_factory->addUnused(unusedObject);
		}
	private:
		/// ReuseFactory with the collection of unused objects for reuse.
		ReuseFactory* m_factory;
	};

	/// Adds an object to the stack of unused objects.
	/// This should only be called from Deleter.
	/// \param unusedObject Object that is no longer referenced by any shared pointers
	void addUnused(T* unusedObject)
	{
		m_unusedObjects.push(std::unique_ptr<T>(unusedObject));
	}

	/// Stack of objects that are available for reuse.
	std::stack<std::unique_ptr<T>> m_unusedObjects;

	Deleter deleter;
};



};  // namespace Framework

};  // namespace SurgSim

#endif  // SURGSIM_FRAMEWORK_REUSEFACTORY_H
