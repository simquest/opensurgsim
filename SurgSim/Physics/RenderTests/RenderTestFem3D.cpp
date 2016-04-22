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

///\file RenderTestFem3D.cpp render test for Fem3D

#include <memory>

#include "SurgSim/Blocks/TransferPhysicsToPointCloudBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DElementCube.h"
#include "SurgSim/Physics/Fem3DElementTetrahedron.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"

using SurgSim::Blocks::TransferPhysicsToPointCloudBehavior;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Graphics::OsgPointCloudRepresentation;
using SurgSim::Physics::Fem3DRepresentation;
using SurgSim::Physics::FemElement;
using SurgSim::Physics::Fem3DElementCube;
using SurgSim::Physics::Fem3DElementTetrahedron;
using SurgSim::Math::Vector3d;

namespace
{

std::shared_ptr<SurgSim::Framework::SceneElement> createTetrahedronFem3D(const std::string& name,
		const SurgSim::Math::RigidTransform3d& pose, SurgSim::Math::Vector4d color,
		SurgSim::Math::IntegrationScheme integrationScheme)
{
	// Physics Representation
	std::shared_ptr<Fem3DRepresentation> physicsRepresentation;
	physicsRepresentation = std::make_shared<Fem3DRepresentation>(name + " Physics");
	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(5e-2);
	physicsRepresentation->setRayleighDampingStiffness(5e-3);

	std::array<Vector3d, 8> vertices = {{
			Vector3d(-0.5, -0.5, -0.5),
			Vector3d(0.5, -0.5, -0.5),
			Vector3d(-0.5,  0.5, -0.5),
			Vector3d(0.5,  0.5, -0.5),
			Vector3d(-0.5, -0.5,  0.5),
			Vector3d(0.5, -0.5,  0.5),
			Vector3d(-0.5,  0.5,  0.5),
			Vector3d(0.5,  0.5,  0.5)
		}
	};

	// Cube decomposition into 5 tetrahedrons
	// https://www.math.ucdavis.edu/~deloera/CURRENT_INTERESTS/cube.html
	std::array< std::array<size_t, 4>, 5> tetrahedrons = {{
			{{4, 7, 1, 2}}, // CCW (47)cross(41) . (42) > 0
			{{4, 1, 7, 5}}, // CCW (41)cross(47) . (45) > 0
			{{4, 2, 1, 0}}, // CCW (42)cross(41) . (40) > 0
			{{4, 7, 2, 6}}, // CCW (47)cross(42) . (46) > 0
			{{1, 2, 7, 3}}  // CCW (12)cross(17) . (13) > 0
		}
	};

	std::array<size_t, 4> boundaryConditionsNodeIdx = {{0, 1, 2, 3}};

	std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
	initialState->setNumDof(physicsRepresentation->getNumDofPerNode(), 8);

	for (size_t i = 0; i != vertices.size(); i++)
	{
		initialState->getPositions().segment(i * 3, 3) = vertices[i];
	}

	for (auto index = boundaryConditionsNodeIdx.cbegin(); index != boundaryConditionsNodeIdx.cend(); ++index)
	{
		initialState->addBoundaryCondition(*index);
	}
	physicsRepresentation->setInitialState(initialState);

	for (auto tetrahedron = tetrahedrons.cbegin(); tetrahedron != tetrahedrons.cend(); ++tetrahedron)
	{
		std::shared_ptr<FemElement> element = std::make_shared<Fem3DElementTetrahedron>(*tetrahedron);
		element->setMassDensity(8000.0);
		element->setPoissonRatio(0.45);
		element->setYoungModulus(1.0e6);
		physicsRepresentation->addFemElement(element);
	}

	// Graphics Representation
	std::shared_ptr<OsgPointCloudRepresentation> graphicsRepresentation;
	graphicsRepresentation = std::make_shared<OsgPointCloudRepresentation>(name + " Graphics object ");
	graphicsRepresentation->setLocalPose(pose);
	graphicsRepresentation->setColor(color);
	graphicsRepresentation->setPointSize(3.0f);
	graphicsRepresentation->setLocalActive(true);

	// Scene Element
	std::shared_ptr<BasicSceneElement> femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);
	femSceneElement->addComponent(graphicsRepresentation);

	auto physicsToGraphics =
		std::make_shared<TransferPhysicsToPointCloudBehavior>("Physics to Graphics deformable points");
	physicsToGraphics->setSource(physicsRepresentation);
	physicsToGraphics->setTarget(graphicsRepresentation);
	femSceneElement->addComponent(physicsToGraphics);

	return femSceneElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createCubeFem3D(const std::string& name,
		const SurgSim::Math::RigidTransform3d& pose,
		SurgSim::Math::Vector4d color, SurgSim::Math::IntegrationScheme integrationScheme)
{
	// Physics Representation
	std::shared_ptr<Fem3DRepresentation> physicsRepresentation;
	physicsRepresentation = std::make_shared<Fem3DRepresentation>(name + " Physics");
	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(5e-2);
	physicsRepresentation->setRayleighDampingStiffness(5e-3);

	std::array<Vector3d, 8> vertices = {{
			Vector3d(-0.5, -0.5, -0.5),
			Vector3d(0.5, -0.5, -0.5),
			Vector3d(-0.5,  0.5, -0.5),
			Vector3d(0.5,  0.5, -0.5),
			Vector3d(-0.5, -0.5,  0.5),
			Vector3d(0.5, -0.5,  0.5),
			Vector3d(-0.5,  0.5,  0.5),
			Vector3d(0.5,  0.5,  0.5)
		}
	};
	std::array<size_t, 8> cube = {{0, 1, 3, 2, 4, 5, 7, 6}};
	std::array<size_t, 4> boundaryConditionsNodeIdx = {{0, 1, 2, 3}};

	std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
	initialState->setNumDof(physicsRepresentation->getNumDofPerNode(), 8);

	for (size_t i = 0; i != vertices.size(); i++)
	{
		initialState->getPositions().segment(i * 3, 3) = vertices[i];
	}

	for (auto index = boundaryConditionsNodeIdx.cbegin(); index != boundaryConditionsNodeIdx.cend(); ++index)
	{
		initialState->addBoundaryCondition(*index);
	}
	physicsRepresentation->setInitialState(initialState);

	std::shared_ptr<FemElement> element = std::make_shared<Fem3DElementCube>(cube);
	element->setMassDensity(8000.0);
	element->setPoissonRatio(0.45);
	element->setYoungModulus(1.0e6);
	physicsRepresentation->addFemElement(element);

	// Graphics Representation
	std::shared_ptr<OsgPointCloudRepresentation> graphicsRepresentation;
	graphicsRepresentation = std::make_shared<OsgPointCloudRepresentation>(name + " Graphics object ");
	graphicsRepresentation->setLocalPose(pose);
	graphicsRepresentation->setColor(color);
	graphicsRepresentation->setPointSize(3.0f);
	graphicsRepresentation->setLocalActive(true);

	// Scene Element
	std::shared_ptr<BasicSceneElement> femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);
	femSceneElement->addComponent(graphicsRepresentation);

	auto physicsToGraphics =
		std::make_shared<TransferPhysicsToPointCloudBehavior>("Physics to Graphics deformable points");
	physicsToGraphics->setSource(physicsRepresentation);
	physicsToGraphics->setTarget(graphicsRepresentation);
	femSceneElement->addComponent(physicsToGraphics);

	return femSceneElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createMultiLayerCubeFem3D(const std::string& name,
	size_t numSubPointX, size_t numSubPointY, size_t numSubPointZ,
	double sizeX, double sizeY, double sizeZ,
	const SurgSim::Math::RigidTransform3d& pose,
	SurgSim::Math::Vector4d color, SurgSim::Math::IntegrationScheme integrationScheme)
{
	// Physics Representation
	std::shared_ptr<Fem3DRepresentation> physicsRepresentation;
	physicsRepresentation = std::make_shared<Fem3DRepresentation>(name + " Physics");
	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(5e-2);
	physicsRepresentation->setRayleighDampingStiffness(5e-3);

	std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
	{
		std::vector<Vector3d> vertices;
		vertices.reserve((2 + numSubPointX) * (2 + numSubPointY) * (2 + numSubPointZ));
		double deltaX = sizeX / static_cast<double>(numSubPointX + 1);
		double deltaY = sizeY / static_cast<double>(numSubPointY + 1);
		double deltaZ = sizeZ / static_cast<double>(numSubPointZ + 1);
		for (size_t k = 0; k <= numSubPointZ + 1; k++)
		{
			double Z = -sizeZ / 2.0 + k * deltaZ;

			for (size_t j = 0; j <= numSubPointY + 1; j++)
			{
				double Y = -sizeY / 2.0 + j * deltaY;

				for (size_t i = 0; i <= numSubPointX + 1; i++)
				{
					double X = -sizeX / 2.0 + i * deltaX;

					vertices.push_back(Vector3d(X, Y, Z));
				}
			}
		}

		std::vector<std::array<size_t, 8>> cubes; //= { { 0, 1, 3, 2, 4, 5, 7, 6 } };
		for (size_t k = 0; k < numSubPointZ + 1; k++)
		{
			size_t iDeltaZ = (2 + numSubPointY) * (2 + numSubPointX);

			for (size_t j = 0; j < numSubPointY + 1; j++)
			{
				size_t iDeltaY = (2 + numSubPointX);

				for (size_t i = 0; i < numSubPointX + 1; i++)
				{
					size_t iDeltaX = 1;

					size_t base = i * iDeltaX + j * iDeltaY + k * iDeltaZ;

					cubes.push_back({{
							base, base + iDeltaX, base + iDeltaX + iDeltaY, base + iDeltaY,
							base + iDeltaZ, base + iDeltaX + iDeltaZ, base + iDeltaX + iDeltaY + iDeltaZ, base + iDeltaY + iDeltaZ,
						}});
				}
			}
		}

		std::vector<size_t> boundaryConditionsNodeIdx;// = { {0, 1, 2, 3 } };
		size_t j = numSubPointY + 1;
		size_t iDeltaY = (2 + numSubPointX);
		for (size_t k = 0; k <= numSubPointZ + 1; k++)
		{
			size_t iDeltaZ = (2 + numSubPointY) * (2 + numSubPointX);

			for (size_t i = 0; i <= numSubPointX + 1; i++)
			{
				size_t iDeltaX = 1;

				boundaryConditionsNodeIdx.push_back(i * iDeltaX + j * iDeltaY + k * iDeltaZ);
			}
		}

		initialState->setNumDof(physicsRepresentation->getNumDofPerNode(), vertices.size());
		for (size_t i = 0; i != vertices.size(); i++)
		{
			initialState->getPositions().segment<3>(i * 3) = vertices[i];
		}
		for (auto index = boundaryConditionsNodeIdx.cbegin(); index != boundaryConditionsNodeIdx.cend(); ++index)
		{
			initialState->addBoundaryCondition(*index);
		}
		for (size_t i = 0; i != cubes.size(); i++)
		{
			std::shared_ptr<FemElement> element = std::make_shared<Fem3DElementCube>(cubes[i]);
			element->setMassDensity(8000.0);
			element->setPoissonRatio(0.45);
			element->setYoungModulus(1.0e6);
			physicsRepresentation->addFemElement(element);
		}
		physicsRepresentation->setInitialState(initialState);
	}

	// Graphics Representation
	std::shared_ptr<OsgPointCloudRepresentation> graphicsRepresentation;
	graphicsRepresentation = std::make_shared<OsgPointCloudRepresentation>(name + " Graphics object ");
	graphicsRepresentation->setLocalPose(pose);
	graphicsRepresentation->setColor(color);
	graphicsRepresentation->setPointSize(3.0f);
	graphicsRepresentation->setLocalActive(true);

	// Scene Element
	std::shared_ptr<BasicSceneElement> femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);
	femSceneElement->addComponent(graphicsRepresentation);

	auto physicsToGraphics =
		std::make_shared<TransferPhysicsToPointCloudBehavior>("Physics to Graphics deformable points");
	physicsToGraphics->setSource(physicsRepresentation);
	physicsToGraphics->setTarget(graphicsRepresentation);
	femSceneElement->addComponent(physicsToGraphics);

	return femSceneElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createMultiLayerTetFem3D(const std::string& name,
	size_t numSubPointX, size_t numSubPointY, size_t numSubPointZ,
	double sizeX, double sizeY, double sizeZ,
	const SurgSim::Math::RigidTransform3d& pose,
	SurgSim::Math::Vector4d color, SurgSim::Math::IntegrationScheme integrationScheme)
{
	// Physics Representation
	std::shared_ptr<Fem3DRepresentation> physicsRepresentation;
	physicsRepresentation = std::make_shared<Fem3DRepresentation>(name + " Physics");
	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(5e-2);
	physicsRepresentation->setRayleighDampingStiffness(5e-3);

	std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
	{
		std::vector<Vector3d> vertices;
		vertices.reserve((2 + numSubPointX) * (2 + numSubPointY) * (2 + numSubPointZ));
		double deltaX = sizeX / static_cast<double>(numSubPointX + 1);
		double deltaY = sizeY / static_cast<double>(numSubPointY + 1);
		double deltaZ = sizeZ / static_cast<double>(numSubPointZ + 1);
		for (size_t k = 0; k <= numSubPointZ + 1; k++)
		{
			double Z = -sizeZ / 2.0 + k * deltaZ;

			for (size_t j = 0; j <= numSubPointY + 1; j++)
			{
				double Y = -sizeY / 2.0 + j * deltaY;

				for (size_t i = 0; i <= numSubPointX + 1; i++)
				{
					double X = -sizeX / 2.0 + i * deltaX;

					vertices.push_back(Vector3d(X, Y, Z));
				}
			}
		}

		std::vector<std::array<size_t, 8>> cubes; //= { { 0, 1, 3, 2, 4, 5, 7, 6 } };
		for (size_t k = 0; k < numSubPointZ + 1; k++)
		{
			size_t iDeltaZ = (2 + numSubPointY) * (2 + numSubPointX);

			for (size_t j = 0; j < numSubPointY + 1; j++)
			{
				size_t iDeltaY = (2 + numSubPointX);

				for (size_t i = 0; i < numSubPointX + 1; i++)
				{
					size_t iDeltaX = 1;

					size_t base = i * iDeltaX + j * iDeltaY + k * iDeltaZ;

					cubes.push_back({ {
							base, base + iDeltaX, base + iDeltaX + iDeltaY, base + iDeltaY,
							base + iDeltaZ, base + iDeltaX + iDeltaZ, base + iDeltaX + iDeltaY + iDeltaZ, base + iDeltaY + iDeltaZ,
						} });
				}
			}
		}

		std::vector<size_t> boundaryConditionsNodeIdx;// = { {0, 1, 2, 3 } };
		size_t j = numSubPointY + 1;
		size_t iDeltaY = (2 + numSubPointX);
		for (size_t k = 0; k <= numSubPointZ + 1; k++)
		{
			size_t iDeltaZ = (2 + numSubPointY) * (2 + numSubPointX);

			for (size_t i = 0; i <= numSubPointX + 1; i++)
			{
				size_t iDeltaX = 1;

				boundaryConditionsNodeIdx.push_back(i * iDeltaX + j * iDeltaY + k * iDeltaZ);
			}
		}

		initialState->setNumDof(physicsRepresentation->getNumDofPerNode(), vertices.size());
		for (size_t i = 0; i != vertices.size(); i++)
		{
			initialState->getPositions().segment<3>(i * 3) = vertices[i];
		}
		for (auto index = boundaryConditionsNodeIdx.cbegin(); index != boundaryConditionsNodeIdx.cend(); ++index)
		{
			initialState->addBoundaryCondition(*index);
		}
		for (size_t i = 0; i != cubes.size(); i++)
		{
			auto c = cubes[i];
			// Cube decomposition into 5 tetrahedrons
			// https://www.math.ucdavis.edu/~deloera/CURRENT_INTERESTS/cube.html
			std::array<std::array<size_t, 4>, 5> tetrahedrons = { {
				{ { c[4], c[6], c[1], c[3] } }, // CCW (47)cross(41) . (42) > 0
				{ { c[4], c[1], c[6], c[5] } }, // CCW (41)cross(47) . (45) > 0
				{ { c[4], c[3], c[1], c[0] } }, // CCW (42)cross(41) . (40) > 0
				{ { c[4], c[6], c[3], c[7] } }, // CCW (47)cross(42) . (46) > 0
				{ { c[1], c[3], c[6], c[2] } }  // CCW (12)cross(17) . (13) > 0
				}
			};

			for (size_t tetId = 0; tetId < 5; tetId++)
			{
				auto t = tetrahedrons[tetId];
				std::shared_ptr<FemElement> element = std::make_shared<Fem3DElementTetrahedron>(t);
				element->setMassDensity(8000.0);
				element->setPoissonRatio(0.45);
				element->setYoungModulus(1.0e6);
				physicsRepresentation->addFemElement(element);
			}
		}
		physicsRepresentation->setInitialState(initialState);
	}

	// Graphics Representation
	std::shared_ptr<OsgPointCloudRepresentation> graphicsRepresentation;
	graphicsRepresentation = std::make_shared<OsgPointCloudRepresentation>(name + " Graphics object ");
	graphicsRepresentation->setLocalPose(pose);
	graphicsRepresentation->setColor(color);
	graphicsRepresentation->setPointSize(3.0f);
	graphicsRepresentation->setLocalActive(true);

	// Scene Element
	std::shared_ptr<BasicSceneElement> femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);
	femSceneElement->addComponent(graphicsRepresentation);

	auto physicsToGraphics =
		std::make_shared<TransferPhysicsToPointCloudBehavior>("Physics to Graphics deformable points");
	physicsToGraphics->setSource(physicsRepresentation);
	physicsToGraphics->setTarget(graphicsRepresentation);
	femSceneElement->addComponent(physicsToGraphics);

	return femSceneElement;
}

}; // anonymous namespace

namespace SurgSim
{

namespace Physics
{

TEST_F(RenderTests, VisualTestFem3D)
{
	using SurgSim::Math::makeRigidTranslation;
	using SurgSim::Math::Vector4d;

	// Cube with cube FemElement
	scene->addSceneElement(createCubeFem3D("CubeElement Euler Explicit",
										   makeRigidTranslation(Vector3d(-4.0, 2.0, -2.0)),
										   Vector4d(1, 0, 0, 1),
										   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_EXPLICIT));

	scene->addSceneElement(createCubeFem3D("CubeElement Modified Euler Explicit",
										   makeRigidTranslation(Vector3d(-2.0, 2.0, -2.0)),
										   Vector4d(0.5, 0, 0, 1),
										   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_EXPLICIT_MODIFIED));

	scene->addSceneElement(createCubeFem3D("CubeElement Runge Kutta 4",
										   makeRigidTranslation(Vector3d(0.0, 2.0, -2.0)),
										   Vector4d(0, 1, 0, 1),
										   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4));

	scene->addSceneElement(createCubeFem3D("CubeElement Fem 3D Euler Implicit",
										   makeRigidTranslation(Vector3d(2.0, 2.0, -2.0)),
										   Vector4d(0, 0, 1, 1),
										   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT));

	scene->addSceneElement(createCubeFem3D("CubeElement Static",
										   makeRigidTranslation(Vector3d(4.0, 2.0, -2.0)),
										   Vector4d(1, 1, 1, 1),
										   SurgSim::Math::INTEGRATIONSCHEME_STATIC));

	// Cube with tetrahedron FemElement
	scene->addSceneElement(createTetrahedronFem3D("TetrahedronElement Euler Explicit",
						   makeRigidTranslation(Vector3d(-4.0, -2.0, -2.0)),
						   Vector4d(1, 0, 0, 1),
						   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_EXPLICIT));

	scene->addSceneElement(createTetrahedronFem3D("TetrahedronElement Modified Euler Explicit",
						   makeRigidTranslation(Vector3d(-2.0, -2.0, -2.0)),
						   Vector4d(0.5, 0, 0, 1),
						   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_EXPLICIT_MODIFIED));

	scene->addSceneElement(createTetrahedronFem3D("TetrahedronElement Runge Kutta 4",
						   makeRigidTranslation(Vector3d(0.0, -2.0, -2.0)),
						   Vector4d(0, 1, 0, 1),
						   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4));

	scene->addSceneElement(createTetrahedronFem3D("TetrahedronElement Fem 3D Euler Implicit",
						   makeRigidTranslation(Vector3d(2.0, -2.0, -2.0)),
						   Vector4d(0, 0, 1, 1),
						   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT));

	scene->addSceneElement(createTetrahedronFem3D("TetrahedronElement Static",
						   makeRigidTranslation(Vector3d(4.0, -2.0, -2.0)),
						   Vector4d(1, 1, 1, 1),
						   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_STATIC));

	runTest(Vector3d(0.0, 0.0, 7.0), Vector3d::Zero(), 5000.0);
}

TEST_F(RenderTests, VisualTestFem3DSubdision)
{
	using SurgSim::Math::makeRigidTranslation;
	using SurgSim::Math::Vector4d;

	// Cube with cube FemElement
	scene->addSceneElement(createMultiLayerCubeFem3D("CubeElement Fem 3D Euler Implicit",
		5, 5, 5, // Number of sub points per dimension (0 => a unique cube)
		1.0, 1.0, 1.0, // dimension of the cube
		makeRigidTranslation(Vector3d(1.0, 1.0, -1.0)),
		Vector4d(0, 0, 1, 1),
		SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT));

	// Cube with tetrahedron FemElement
	scene->addSceneElement(createMultiLayerTetFem3D("TetrahedronElement Fem 3D Euler Implicit",
		5, 5, 5, // Number of sub points per dimension (0 => a unique cube)
		1.0, 1.0, 1.0, // dimension of the cube
		makeRigidTranslation(Vector3d(-1.0, 1.0, -1.0)),
		Vector4d(0, 0, 1, 1),
		SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT));

	runTest(Vector3d(0.0, 0.0, 4.0), Vector3d::Zero(), 10000.0);
}

}; // namespace Physics

}; // namespace SurgSim
