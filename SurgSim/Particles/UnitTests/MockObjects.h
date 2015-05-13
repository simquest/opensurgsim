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

#ifndef SURGSIM_PARTICLES_UNITTESTS_MOCKOBJECTS_H
#define SURGSIM_PARTICLES_UNITTESTS_MOCKOBJECTS_H

#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Particles/Emitter.h"
#include "SurgSim/Particles/Representation.h"


class MockParticleSystem : public SurgSim::Particles::Representation
{
public:
	explicit MockParticleSystem(const std::string& name);
	SURGSIM_CLASSNAME(MockParticleSystem);
	int updateCount;

private:
	bool doUpdate(double dt) override;
};

class MockEmitter : public SurgSim::Particles::Emitter
{
public:
	explicit MockEmitter(const std::string& name);
	void update(double dt) override;
	int updateCount;
};

#endif //SURGSIM_PARTICLES_UNITTESTS_MOCKOBJECTS_H


