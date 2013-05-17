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

#include "Sphere.h"

#include "SurgSim/Framework/Behavior.h"

class PoseUpdateBehavior : public SurgSim::Framework::Behavior
{
	PoseUpdateBehavior();
	~PoseUpdateBehavior();
};



Sphere::Sphere(const std::string& name) : SceneElement(name)
{

}

Sphere::~Sphere()
{

}

bool Sphere::doInitialize()
{
	// Create Rigid Actor
	// Create Sphere Shape for Rigid Actor
	// Create Graphics Sphere
	// Create Update Behavior
	// Add all of them ...
	// Connect RigidActor and Graphics Sphere in Update Behavior
	
	return false;
}

bool Sphere::doWakeUp()
{
	return true;
}
