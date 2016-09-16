// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_FRAMEWORK_TRANSFERPROPERTIESBEHAVIOR_H
#define SURGSIM_FRAMEWORK_TRANSFERPROPERTIESBEHAVIOR_H

#include <boost/thread/mutex.hpp>
#include <string>

#include "SurgSim/Framework/Accessible.h"
#include "SurgSim/Framework/Behavior.h"


namespace SurgSim
{
namespace Framework
{

/// Behavior to copy properties between instances of Accessible
/// \note HS-2013-dec-12 does not support removal of connections yes
class TransferPropertiesBehavior : public SurgSim::Framework::Behavior
{
public:

	/// Constructor.
	/// \param name The name of the behavior.
	explicit TransferPropertiesBehavior(const std::string& name);

	/// Destructor.
	virtual ~TransferPropertiesBehavior();

	/// Connect two properties of two instances of accessible, once connected the value of the property
	/// will be copied from source to target at every update call.
	/// \pre pointers cannot be nullptr, properties need to exist and the property
	///      at the source needs to be readable and the property at the target needs
	///      to be writeable
	/// \param sourceAccessible Source Accessible instance.
	/// \param sourcePropertyName The name of the source property.
	/// \param tagetAccessible Target Accessible instance.
	/// \param targetPropertyName The name of the target property.
	/// \return true if the connection was created
	bool connect(
		std::shared_ptr<SurgSim::Framework::Accessible> sourceAccessible,
		const std::string& sourcePropertyName,
		std::shared_ptr<SurgSim::Framework::Accessible> tagetAccessible,
		const std::string& targetPropertyName);

	/// Connect two properties of two instances of accessible, once connected the value of the property
	/// will be copied from source to target at every update call.
	/// \pre pointers cannot be nullptr, properties need to exist and the property
	///      at the source needs to be readable and the property at the target needs
	///      to be writeable
	/// \param source Source property.
	/// \param target Target property.
	/// \return true if the connection was created
	bool connect(const Property& source, const Property& target);

	/// Sets the type of manager that this behavior should use, this cannot be done after
	/// initialization has occurred.
	/// \param managerType Type of manager for this behavior
	void setTargetManagerType(int managerType);

	int getTargetManagerType() const override;

	void update(double dt) override;

private:

	///@{
	/// Overridden from Behavior
	bool doInitialize() override;
	bool doWakeUp() override;
	///@}

	/// Local typedefs
	typedef std::pair<Property, Property> Connection;

	/// List of connections in this object
	std::vector<Connection> m_connections;

	/// Lock for adding new connections
	boost::mutex m_incomingMutex;

	/// Queue for adding new connections
	std::vector<Connection> m_incomingConnections;

	/// The manager type that will handle this behavior
	int m_targetManager;
};

}
}

#endif
