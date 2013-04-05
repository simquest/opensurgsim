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

#ifndef SURGSIM_PHYSICS_ACTOR_H
#define SURGSIM_PHYSICS_ACTOR_H

#include <SurgSim/Framework/Representation.h>

#include <memory>
#include <string>

namespace SurgSim 
{

namespace Physics
{

class ActorImplementation;

class Actor : public SurgSim::Framework::Representation
{
public:
	Actor(const std::string& name, std::shared_ptr<ActorImplementation> implementation);
	virtual ~Actor();

	/// Query if this object is active in the scene.
	/// \return	true if active, false if not.
	bool isActive() 
	{
		return doIsActive();
	}

	/// Called before the scene does its physics update, lets the actor
	/// do some preprocessing
	/// \param	dt	The time in seconds that the update call will advance the scene.
	void beforeUpdate(double dt)
	{
		doBeforeUpdate(dt);
	}

	/// Called after the scene update has concluded.
	/// \param	dt	The time in seconds of the preceding timestep.
	void afterUpdate(double dt) 
	{
		doAfterUpdate(dt);
	}

	std::shared_ptr<ActorImplementation> getImplementation()
	{
		return m_implementation;
	}

private:
	virtual bool doIsActive() = 0;

	// By Default do nothing
	virtual void doBeforeUpdate(double dt) {};
	virtual void doAfterUpdate(double dt) {};

	std::string m_name;
	std::shared_ptr<ActorImplementation> m_implementation;
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_ACTOR_H
