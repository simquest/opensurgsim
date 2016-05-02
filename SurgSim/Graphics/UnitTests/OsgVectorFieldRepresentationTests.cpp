// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

/// \file
/// Unit Tests for the OsgVectorFieldRepresentation class.

#include "SurgSim/Graphics/VectorFieldRepresentation.h"
#include "SurgSim/Graphics/OsgVectorFieldRepresentation.h"
#include "SurgSim/Math/Vector.h"

#include <gtest/gtest.h>

namespace
{
const double epsilon = 1e-6;
};

using SurgSim::Graphics::OsgVectorFieldRepresentation;
using SurgSim::Graphics::VectorFieldRepresentation;

TEST(OsgVectorFieldRepresentationTests, VerticesTest)
{
	std::shared_ptr<VectorFieldRepresentation> vectorFieldRepresentation =
		std::make_shared<OsgVectorFieldRepresentation>("Vector Field");

	EXPECT_EQ(0u, vectorFieldRepresentation->getVectorField()->getNumVertices());
}

TEST(OsgVectorFieldRepresentationTests, LineWidthTest)
{
	std::shared_ptr<VectorFieldRepresentation> vectorFieldRepresentation =
		std::make_shared<OsgVectorFieldRepresentation>("Vector Field");
	vectorFieldRepresentation->setLineWidth(1.25);
	EXPECT_NEAR(1.25, vectorFieldRepresentation->getLineWidth(), epsilon);
}

TEST(OsgVectorFieldRepresentationTests, ScaleTest)
{
	std::shared_ptr<VectorFieldRepresentation> vectorFieldRepresentation =
		std::make_shared<OsgVectorFieldRepresentation>("Vector Field");
	vectorFieldRepresentation->setScale(1.25);
	EXPECT_NEAR(1.25, vectorFieldRepresentation->getScale(), epsilon);
}

TEST(OsgVectorFieldRepresentationTests, PointSizeTest)
{
	std::shared_ptr<VectorFieldRepresentation> vectorFieldRepresentation =
		std::make_shared<OsgVectorFieldRepresentation>("Vector Field");
	vectorFieldRepresentation->setPointSize(1.25);
	EXPECT_NEAR(1.25, vectorFieldRepresentation->getPointSize(), epsilon);
}

TEST(OsgVectorFieldRepresentation, Properties)
{
	auto representation = std::make_shared<OsgVectorFieldRepresentation>("Vector Field");
	EXPECT_EQ("SurgSim::Graphics::OsgVectorFieldRepresentation", representation->getClassName());

	EXPECT_NO_THROW(representation->setValue("Scale", 2.34));
	EXPECT_NEAR(2.34, representation->getValue<double>("Scale"), epsilon);

	EXPECT_NO_THROW(representation->setValue("LineWidth", 4.56));
	EXPECT_NEAR(4.56, representation->getValue<double>("LineWidth"), epsilon);

	EXPECT_NO_THROW(representation->setValue("PointSize", 7.89));
	EXPECT_NEAR(7.89, representation->getValue<double>("PointSize"), epsilon);

}