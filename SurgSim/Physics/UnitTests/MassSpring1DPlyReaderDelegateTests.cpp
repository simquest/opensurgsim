// This file is a part of the OpenSurgSim project.
// Copyright 2015-2016, SimQuest Solutions Inc.
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

#include <gtest/gtest.h>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/MassSpring1DPlyReaderDelegate.h"

namespace SurgSim
{
namespace Physics
{
using SurgSim::Math::Vector3d;
using SurgSim::DataStructures::PlyReader;

TEST(MassSpring1DRepresentationReaderTests, DelegateTest)
{
	auto massSpringRep = std::make_shared<MassSpringRepresentation>("rep");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	ASSERT_TRUE(massSpringRep->load1DMassSpringFile("PlyReaderTests/MassSpring1D.ply"));
}

}; // Physics
}; // SurgSim
