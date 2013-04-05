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

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <SurgSim/Math/Vector.h>

namespace SurgSim
{

namespace Physics
{

class Actor;
class ActorState;

class Localization
{
public:
	Localization();
	Localization(std::shared_ptr<Actor> actor);
	virtual ~Localization();

	void setActor(std::shared_ptr<Actor> actor)
	{
		m_actor = actor;
	}
	std::shared_ptr<Actor> getActor() const
	{
		return m_actor;
	}

	SurgSim::Math::Vector3d calculatePosition(const ActorState& state)
	{
		doCalculatePosition(state);
	}

	bool operator==(const Localization& localization) const;

	bool operator!=(const Localization& localization) const;

private:
	virtual bool isEqual(const Localization& localization) const = 0;

	virtual SurgSim::Math::Vector3d doCalculatePosition(const ActorState& state) = 0;

	std::shared_ptr<Actor> m_actor;
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // LOCALIZATION_H
