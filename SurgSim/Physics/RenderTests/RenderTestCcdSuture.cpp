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

///\file RenderTestCcdSuture.cpp render test for ccd with Fem1D

#include <memory>

#include "SurgSim/Blocks/Blocks.h"
#include "SurgSim/Blocks/VisualizeContactsBehavior.h"
#include "SurgSim/Blocks/VisualizeConstraints.h"
#include "SurgSim/Framework/Framework.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Graphics/Graphics.h"
#include "SurgSim/Math/Math.h"
#include "SurgSim/Physics/Physics.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"


using SurgSim::DataStructures::SegmentMeshPlain;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Graphics::OsgAxesRepresentation;
using SurgSim::Graphics::OsgMeshRepresentation;
using SurgSim::Math::SegmentMeshShape;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::Fem1DElementBeam;
using SurgSim::Physics::Fem1DLocalization;
using SurgSim::Physics::Fem1DRepresentation;
using SurgSim::Physics::FemElement;


namespace
{

	void extendControlPoints(
		const std::vector<SurgSim::Math::Vector3d>& points,
		std::vector<SurgSim::Math::Vector3d>* result)
	{
		SURGSIM_ASSERT(points.size() >= 2) << "Cannot apply CatmullRom with less than 2 points";
		result->clear();
		result->reserve(points.size() + 2);

		// Interpolate the 1st point (ghost) as the symmetric of P1 from P0: P-1 = P0 + P1P0
		result->push_back(2.0 * points[0] - points[1]);
		for (size_t i = 0; i < points.size(); ++i)
		{
			result->push_back(points[i]);
		}
		// Interpolate the last point (ghost) as the symmetric of Pn-1 from Pn: Pn+1 = Pn + Pn-1Pn
		result->push_back(2.0 * points[points.size() - 1] - points[points.size() - 2]);
	}

	// Interpolate the segment smoothly using Cardinal splines.
	// \param subdivisions are the number of subdivisions to divide the interval
	// \param controlPoint are the points through which the spline passes
	// \param points [out] interpolation points
	// \param tau tension parameter
	void interpolate(int subdivisions,
		const std::vector<SurgSim::Math::Vector3d>& controlPoints,
		std::vector<SurgSim::Math::Vector3d>* points,
		double tau = 0.4)
	{
		size_t numPoints = controlPoints.size();
		double stepsize = 1.0 / static_cast<double>(subdivisions);
		size_t pointIndex = 0;
		while (pointIndex < numPoints - 3)
		{
			std::array<SurgSim::Math::Vector3d, 4> p =
			{
				controlPoints[pointIndex],
				controlPoints[pointIndex + 1],
				controlPoints[pointIndex + 2],
				controlPoints[pointIndex + 3]
			};

			double abscissa = 0.0;
			while (abscissa < 1.0)
			{
				double abcissaSquared = abscissa * abscissa;
				double abcissaCubed = abcissaSquared * abscissa;

				SurgSim::Math::Vector3d result =
					p[1] +
					abscissa * (tau * (p[2] - p[0])) +
					abcissaSquared * (2.0 * tau * p[0] + (tau - 3.0) * p[1] + (3.0 - 2.0 * tau) * p[2] - tau * p[3]) +
					abcissaCubed * (-tau * p[0] + (2.0 - tau) * p[1] + (tau - 2.0) * p[2] + tau * p[3]);

				points->push_back(std::move(result));

				abscissa += stepsize;
			}
			++pointIndex;
		}
	}

	std::shared_ptr<SurgSim::Framework::SceneElement> makeSuture(const std::string& filename)
	{
		auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Suture");

		// Physics
		auto physics = std::make_shared<SurgSim::Physics::Fem1DRepresentation>("Physics");
		physics->setFemElementType("SurgSim::Physics::Fem1DElementBeam");
		physics->setLocalPose(SurgSim::Math::RigidTransform3d::Identity());
		physics->loadFem(filename);
		physics->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_EULER_IMPLICIT);
		physics->setLinearSolver(SurgSim::Math::LINEARSOLVER_LU);
		physics->setRayleighDampingMass(5.0);
		physics->setRayleighDampingStiffness(0.001);
		physics->setIsGravityEnabled(true);
		element->addComponent(physics);

		// Graphics
		auto gfx = std::make_shared<SurgSim::Graphics::OsgCurveRepresentation>("Graphics");
		gfx->setColor(SurgSim::Math::Vector4d(0.0, 0.0, 1.0, 1.0));
		gfx->setAntiAliasing(true);
		gfx->setWidth(0.9);
		element->addComponent(gfx);

		auto collision = std::make_shared<SurgSim::Physics::DeformableCollisionRepresentation>("Collision");
		auto shape = std::make_shared<SurgSim::Math::SegmentMeshShape>();
		shape->load(filename);
		shape->setRadius(0.0001);
		collision->setShape(shape);
		collision->setCollisionDetectionType(SurgSim::Collision::COLLISION_DETECTION_TYPE_CONTINUOUS);
		collision->setSelfCollisionDetectionType(SurgSim::Collision::COLLISION_DETECTION_TYPE_CONTINUOUS);
		physics->setCollisionRepresentation(collision);
		element->addComponent(collision);

		auto copier = std::make_shared<SurgSim::Blocks::TransferPhysicsToVerticesBehavior>("Copier");
		copier->setSource(physics);
		copier->setTarget(gfx);
		element->addComponent(copier);

		return element;
	}

	std::shared_ptr<SurgSim::Framework::SceneElement> makeRigid(const std::string& filename)
	{
		SurgSim::Math::RigidTransform3d pose = SurgSim::Math::makeRigidTranslation(SurgSim::Math::Vector3d(0.0, -0.1, 0.0));

		auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("RigidMesh");
		element->setPose(pose);
		auto shape = std::make_shared<SurgSim::Math::MeshShape>();
		shape->load(filename);

		auto rigid = std::make_shared<SurgSim::Physics::RigidRepresentation>("Physics");
		rigid->setIsGravityEnabled(false);
		// http://www.engineeringtoolbox.com/wood-density-d_40.html
		rigid->setDensity(5800.0); // Cedar of Lebanon wood density 5800.0 Kg/m-3
		rigid->setShape(shape);
		element->addComponent(rigid);

		auto collision = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("Collision");
		collision->setCollisionDetectionType(SurgSim::Collision::COLLISION_DETECTION_TYPE_CONTINUOUS);
		rigid->setCollisionRepresentation(collision);
		collision->setShape(shape);
		element->addComponent(collision);

		std::shared_ptr<OsgMeshRepresentation> osgRepresentation =
			std::make_shared<OsgMeshRepresentation>("Graphics");
		osgRepresentation->setShape(shape);
		element->addComponent(osgRepresentation);

		return element;
	}

}

class CcdSutureTest : public SurgSim::Physics::RenderTests
{
public:
	void SetUp() override
	{
		SurgSim::Physics::RenderTests::SetUp();

		SurgSim::Framework::Logger::getLoggerManager()->setThreshold(SurgSim::Framework::LOG_LEVEL_DEBUG);
		physicsManager->setRate(150.0);
	}

protected:
	std::shared_ptr<SurgSim::Physics::RigidRepresentationBase> m_phxRigid;

	std::shared_ptr<SurgSim::Framework::SceneElement> m_rigidSceneElement;
};

TEST_F(CcdSutureTest, SutureVsMeshedCylinder)
{
	SurgSim::Math::Vector3d cameraPosition(0.25, 0.0, 0.1);
	SurgSim::Math::Vector3d cameraLookAt(0.0, -0.1, 0.0);
	double miliseconds = 25000.0;

	physicsManager->setRate(4000.0);
	physicsManager->setComputations(SurgSim::Physics::createCcdPipeline());

	scene->addSceneElement(std::make_shared<SurgSim::Blocks::VisualizeConstraints>());

	// Load the deformable suture
	scene->addSceneElement(makeSuture("prolene 3.0-fixedExtremity.ply"));
	scene->addSceneElement(makeRigid("cylinder.ply"));

	runTest(cameraPosition, cameraLookAt, miliseconds);
}

TEST_F(CcdSutureTest, Fem1DHalfKnot)
{
	SurgSim::Framework::Logger::getLogger("SegmentSelfContact")->setThreshold(SurgSim::Framework::LOG_LEVEL_INFO);
	scene->addSceneElement(makeSuture("half_knot.ply"));
	physicsManager->setRate(4000.0);
	physicsManager->setComputations(SurgSim::Physics::createCcdPipeline());

	SurgSim::Framework::Logger::getLogger("Collision/SegmentSelfContact")->setThreshold(
		SurgSim::Framework::LOG_LEVEL_WARNING);
	scene->addSceneElement(std::make_shared<SurgSim::Blocks::VisualizeConstraints>());

	SurgSim::Math::Vector3d cameraPosition(0.25, 0.0, 0.25);
	SurgSim::Math::Vector3d cameraLookAt(0.0, 0.0, 0.0);
	double miliseconds = 10000.0;

	runTest(cameraPosition, cameraLookAt, miliseconds);
}

TEST_F(CcdSutureTest, Fem1DLoop)
{
	SurgSim::Framework::Logger::getLogger("SegmentSelfContact")->setThreshold(SurgSim::Framework::LOG_LEVEL_INFO);
	scene->addSceneElement(makeSuture("loop.ply"));
	physicsManager->setRate(1000.0);
	physicsManager->setComputations(SurgSim::Physics::createCcdPipeline());

	SurgSim::Framework::Logger::getLogger("Collision/SegmentSelfContact")->setThreshold(
		SurgSim::Framework::LOG_LEVEL_WARNING);
	scene->addSceneElement(std::make_shared<SurgSim::Blocks::VisualizeConstraints>());

	SurgSim::Math::Vector3d cameraPosition(0.25, 0.0, 0.25);
	SurgSim::Math::Vector3d cameraLookAt(0.0, 0.0, 0.0);
	double miliseconds = 5000.0;

	runTest(cameraPosition, cameraLookAt, miliseconds);
}
