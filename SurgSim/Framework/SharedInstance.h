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

#ifndef SURGSIM_FRAMEWORK_SHAREDINSTANCE_H
#define SURGSIM_FRAMEWORK_SHAREDINSTANCE_H

#include <memory>
#include <functional>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <SurgSim/Framework/Assert.h>


namespace SurgSim
{
namespace Framework
{

/// A shared smart pointer that manages a shared instance of an object.
///
/// This class behaves similarly to the Singleton pattern, in that it manages a single shared instance of an
/// object. However, the object will not only be created as needed, but also destroyed when all of the
/// shared_ptr references to it are released.  If it is released and then needed again, a new instance will be
/// created.
///
/// Code using this class will normally use the get() method to initialize its own shared_ptr referencing the
/// shared instance.  Such code should hold onto the shared_ptr for as long as it needs to use the shared instance.
/// As soon as all of the shared pointers are released, the shared object instance will be destroyed.
///
/// <b>A Simple Example</b>
///
/// To share an instance between many objects in the same class, you can declare a static SharedInstance member:
/// ~~~~
///   class ManyObjects
///   {
///       ManyObjects() : m_sharedInstance(m_sharedInstanceManager.get())
///       {
///       }
///
///       // ...
///
///       std::shared_ptr<SingleObject> m_sharedInstance;
///       static SurgSim::Framework::SharedInstance<SingleObject> m_sharedInstanceManager;
///   }
///
///   // Need to also define the static member somewhere:
///   SurgSim::Framework::SharedInstance<SingleObject> ManyObjects::m_sharedInstanceManager;
/// ~~~~
/// The downside to this approach is that it requires the SharedInstance object itself to be created as a static
/// member of a class.  Since the order of construction (and destruction) of various static data members is not
/// well defined by the C++ language standards, this can lead to objects getting used before they are initialized.
///
/// <b>A Better Example</b>
///
/// To avoid the initialization order problems, you can instead use the SharedInstance as a static variable
/// inside a function or a (most likely also static) method, so it will not be initialized until that
/// function/method is first called.  (Note that this is only safe if your compiler follows the C++11
/// requirement that the initialization of static variables inside functions must be atomic.)
/// ~~~~
///   class ManyObjects
///   {
///       ManyObjects() : m_sharedInstance(getSharedInstance())
///       {
///       }
///
///       // ...
///
///       static std::shared_ptr<SingleObject> getSharedInstance();
///       std::shared_ptr<SingleObject> m_sharedInstance;
///   }
///
///   std::shared_ptr<SingleObject> ManyObjects::getSharedInstance()
///   {
///       static SharedInstance<SingleObject> shared;
///       return shared.get();
///   }
/// ~~~~
///
/// \tparam T Type of the data held by the SharedInstance.
template <typename T>
class SharedInstance
{
public:
	/// Create the SharedInstance object used to manage the shared instance.
	/// Note that this <em>does not</em> immediately create the instance itself.
	/// If and when the shared instance is created, it will be initialized using the default constructor.
	SharedInstance()
	{
	}

	/// Destroy the container and the data it contains.
	~SharedInstance()
	{
	}

	/// Gets the shared object instance.
	/// If the instance has not been created previously, it will be created during the call.
	///
	/// The calling code should generally copy the shared_ptr and hold onto it for as long as needed.
	/// As soon as all of the shared pointers are released, the shared object instance will be destroyed.
	///
	/// \return a shared_ptr holding the instance.
	std::shared_ptr<T> get()
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		std::shared_ptr<T> instance = m_weakInstance.lock();
		if (! instance)
		{
			instance = createInstance();
			m_weakInstance = instance;
		}
		return std::move(instance);
	}

private:
	// /// A type that can hold a function or lambda that takes no arguments and returns std::shared_ptr<T>.
	// typedef std::function<std::shared_ptr<T>()> TypeCreator;

	/// Prevent copying
	SharedInstance(const SharedInstance&);
	/// Prevent assignment
	SharedInstance& operator=(const SharedInstance&);

	std::shared_ptr<T> createInstance()
	{
		std::shared_ptr<T> instance = std::make_shared<T>();
		SURGSIM_ASSERT(instance);
		return std::move(instance);
	}


	/// A weak reference to the shared instance, if any.
	std::weak_ptr<T> m_weakInstance;

	/// Mutex for synchronization of object creation.
	boost::mutex m_mutex;
};

};  // namespace Framework
};  // namespace SurgSim

#endif  // SURGSIM_FRAMEWORK_SHAREDINSTANCE_H
