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
/// Render Tests for the OsgCapsuleRepresentation class.

#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgCapsuleRepresentation.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>

#include <gtest/gtest.h>

#include <random>

using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector2d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;

namespace SurgSim
{

namespace Graphics
{


TEST(OsgCapsuleRepresentationRenderTests, MovingCapsuleTest)
{
    /// Initial capsule 1 position
    Vector3d startPosition1(-0.1, 0.0, -0.2);
    /// Final capsule 1 position
    Vector3d endPosition1(0.1, 0.0, -0.2);
    /// Initial capsule 1 sizeX;
    double startRadius1 = 0.001;
    /// Final capsule 1 sizeX;
    double endRadius1 = 0.01;
    /// Initial capsule 1 sizeY;
    double startHeight1 = 0.011;
    /// Final capsule 1 sizeY;
    double endHeight1 = 0.02;
    /// Initial capsule 1 sizeZ;
    double startSizeZ1 = 0.021;
    /// Final capsule 1 sizeZ;
    double endSizeZ1 = 0.03;
    /// Initial capsule 1 angleX;
    double startAngleX1 = 0.0;
    /// Final capsule 1 angleX;
    double endAngleX1 = - M_PI / 4.0;
    /// Initial capsule 1 angleY;
    double startAngleY1 = 0.0;
    /// Final capsule 1 angleY;
    double endAngleY1 = - M_PI / 4.0;
    /// Initial capsule 1 angleZ;
    double startAngleZ1 = 0.0;
    /// Final capsule 1 angleZ;
    double endAngleZ1 = - M_PI / 4.0;
    /// Initial capsule 2 position
    Vector3d startPosition2(0.0, -0.1, -0.2);
    /// Final capsule 2 position
    Vector3d endPosition2(0.0, 0.1, -0.2);
    /// Initial capsule 2 sizeX;
    double startRadius2 = 0.001;
    /// Final capsule 2 sizeX;
    double endRadius2 = 0.01;
    /// Initial capsule 2 sizeX;
    double startHeight2 = 0.011;
    /// Final capsule 2 sizeX;
    double endHeight2 = 0.02;
    /// Initial capsule 2 sizeX;
    double startSizeZ2 = 0.021;
    /// Final capsule 2 sizeX;
    double endSizeZ2 = 0.03;
    /// Initial capsule 2 angleX;
    double startAngleX2 = -M_PI / 2.0;;
    /// Final capsule 2 angleX;
    double endAngleX2 = M_PI;
    /// Initial capsule 2 angleY;
    double startAngleY2 = -M_PI / 2.0;;
    /// Final capsule 2 angleY;
    double endAngleY2 = M_PI;
    /// Initial capsule 2 angleZ;
    double startAngleZ2 = -M_PI / 2.0;;
    /// Final capsule 2 angleZ;
    double endAngleZ2 = M_PI;

    /// Number of times to step the capsule position and radius from start to end.
    /// This number of steps will be done in 1 second.
    int numSteps = 100;

    std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
    std::shared_ptr<OsgManager> manager = std::make_shared<OsgManager>();

    runtime->addManager(manager);

    std::shared_ptr<Scene> scene = std::make_shared<Scene>();
    runtime->setScene(scene);

    /// Add a graphics view element to the scene
    std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>("view element");
    scene->addSceneElement(viewElement);

    /// Add the two capsule representation to the view element so we don't need to make another concrete scene element
    std::shared_ptr<CapsuleRepresentation> capsuleRepresentation1 =
        std::make_shared<OsgCapsuleRepresentation>("capsule representation 1");
    viewElement->addComponent(capsuleRepresentation1);
    std::shared_ptr<CapsuleRepresentation> capsuleRepresentation2 =
        std::make_shared<OsgCapsuleRepresentation>("capsule representation 2");
    viewElement->addComponent(capsuleRepresentation2);

    /// Run the thread
    runtime->start();
    EXPECT_TRUE(manager->isInitialized());
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    enum SetterType {SetterTypeIndividual,
                        SetterTypeTogether,
                        SetterTypeVector2d,
                        // Add more setter types above this line.
                        BoxSetterTypeCount};
    int setterType = 0;
    Vector2d capsule1Size, capsule2Size;

    for (int i = 0; i < numSteps; ++i)
    {
        /// Calculate t in [0.0, 1.0]
        double t = static_cast<double>(i) / numSteps;
        /// Interpolate position and radius
        capsuleRepresentation1->setPose(makeRigidTransform(
            makeRotationQuaternion((1.0 - t) * startAngleX1 + t * endAngleX1,
            Vector3d(1.0, 0.0, 0.0)) * makeRotationQuaternion((1.0 - t) * startAngleY1 + t * endAngleY1,
            Vector3d(0.0, 1.0, 0.0)) * makeRotationQuaternion((1.0 - t) * startAngleZ1 + t * endAngleZ1,
            Vector3d(0.0, 0.0, 1.0)), (1.0 - t) * startPosition1 + t * endPosition1));
        capsuleRepresentation2->setPose(makeRigidTransform(
            makeRotationQuaternion((1.0 - t) * startAngleX2 + t * endAngleX2,
            Vector3d(1.0, 0.0, 0.0)) * makeRotationQuaternion((1.0 - t) * startAngleY2 + t * endAngleY2,
            Vector3d(0.0, 1.0, 0.0)) * makeRotationQuaternion((1.0 - t) * startAngleZ2 + t * endAngleZ2,
            Vector3d(0.0, 0.0, 1.0)), (1.0 - t) * startPosition2 + t * endPosition2));
        if(setterType == static_cast<int>(SetterTypeIndividual))
        {
            capsuleRepresentation1->setRadius((1 - t) * startRadius1 + t * endRadius1);
            capsuleRepresentation1->setHeight((1 - t) * startHeight1 + t * endHeight1);

            capsuleRepresentation2->setRadius((1 - t) * startRadius2 + t * endRadius2);
            capsuleRepresentation2->setHeight((1 - t) * startHeight2 + t * endHeight2);
        }
        else if(setterType == static_cast<int>(SetterTypeTogether))
        {
            capsuleRepresentation1->setSize((1 - t) * startRadius1 + t * endRadius1,
(1 - t) * startHeight1 + t * endHeight1);

            capsuleRepresentation2->setSize((1 - t) * startRadius2 + t * endRadius2,
(1 - t) * startHeight2 + t * endHeight2);
        }
        else if(setterType == static_cast<int>(SetterTypeVector2d))
        {
            capsule1Size.x() = (1 - t) * startRadius1 + t * endRadius1;
            capsule1Size.y() = (1 - t) * startHeight1 + t * endHeight1;
            capsuleRepresentation1->setSize(capsule1Size);

            capsule2Size.x() = (1 - t) * startRadius2 + t * endRadius2;
            capsule2Size.y() = (1 - t) * startHeight2 + t * endHeight2;
            capsuleRepresentation2->setSize(capsule2Size);
        }
        setterType = (setterType + 1) % BoxSetterTypeCount;
        /// The total number of steps should complete in 1 second
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
    }

    runtime->stop();
}

};  // namespace Graphics

};  // namespace SurgSim