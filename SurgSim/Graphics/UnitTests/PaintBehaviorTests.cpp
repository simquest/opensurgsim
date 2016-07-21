// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/PaintBehavior.h"

namespace SurgSim
{
namespace Graphics
{

class PaintBehaviorTests : public testing::Test
{

};

TEST_F(PaintBehaviorTests, Serialization)
{
	auto component = std::make_shared<Graphics::PaintBehavior>("Painter");

	auto representation = std::make_shared<Graphics::OsgMeshRepresentation>("MeshRepresentation");
	auto color = Math::Vector4d(1.0, 0.0, 0.0, 1.0);

	component->setRepresentation(representation);
	component->setColor(color);
	component->setAntiAlias(true);

	YAML::Node node(YAML::convert<SurgSim::Framework::Component>::encode(*component));

	auto decode = std::dynamic_pointer_cast<PaintBehavior>(
					node.as<std::shared_ptr<PaintBehavior>>());

	EXPECT_NE(nullptr, decode);
	EXPECT_EQ(representation->getFullName(), decode->getRepresentation()->getFullName());
	EXPECT_TRUE(color.isApprox(decode->getColor()));
	EXPECT_TRUE(decode->getAntiAlias());
}

} // namespace Graphics
} // namespace SurgSim