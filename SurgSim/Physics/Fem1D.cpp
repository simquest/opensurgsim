// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#include "SurgSim/Physics/Fem1D.h"
#include "SurgSim/Physics/Fem1DPlyReaderDelegate.h"

namespace SurgSim
{
namespace Physics
{

SURGSIM_REGISTER(SurgSim::Framework::Asset, SurgSim::Physics::Fem1D, Fem1D)

Fem1D::Fem1D() : Fem()
{
}

bool Fem1D::doLoad(const std::string& filePath)
{
	return loadFemFile<Fem1DPlyReaderDelegate, Fem1D>(filePath);
}
} // namespace Physics
} // namespace SurgSim
