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

#include "Localization.h"

#include <utility>

using SurgSim::Physics::Actor;
using SurgSim::Physics::Localization;

Localization::Localization()
{
}
Localization::Localization(std::shared_ptr<Actor> actor) : m_actor(actor)
{
}

Localization::~Localization()
{
}

bool Localization::operator==(const Localization& localization) const
{
	return (typeid(*this) == typeid(localization)) && isEqual(localization);
}

bool Localization::operator!=(const Localization& localization) const
{
	return (typeid(*this) != typeid(localization)) || ! isEqual(localization);
}