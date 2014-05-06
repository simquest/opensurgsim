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

#include <memory>

#include <gtest/gtest.h>

#include "SurgSim/Blocks/TransferDeformableStateToVerticesBehavior.h"
#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/DataStructures/TriangleMeshPlyReaderDelegate.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Math/OdeSolver.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationPlyReaderDelegate.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"

using SurgSim::Math::Vector3d;
using SurgSim::DataStructures::EmptyData;

namespace SurgSim
{
namespace Physics
{

template <class SourceType, class TargetType>
class MeshToMeshTransfer : public SurgSim::Framework::Behavior
{
public:
	explicit MeshToMeshTransfer(const std::string& name) : Behavior(name)
	{

	}

	virtual void update(double dt)
	{
		SURGSIM_ASSERT(m_sourceMesh->getNumVertices() == m_targetMesh->getNumVertices()) <<
				"Source and target mesh have differing vertex count, can't use MeshToMeshTransfer.";
		size_t count = m_sourceMesh->getNumVertices();
		for (size_t i = 0; i < count; ++i)
		{
			m_targetMesh->setVertexPosition(i, m_sourceMesh->getVertexPosition(i));
		}
		m_targetMesh->update();
	}

	virtual bool doInitialize()
	{
		return true;
	}

	virtual bool doWakeUp()
	{
		SURGSIM_ASSERT(m_sourceMesh != nullptr) << "SourceMesh cannot be empty.";
		SURGSIM_ASSERT(m_targetMesh != nullptr) << "TargetMesh cannot be empty.";
		SURGSIM_ASSERT(m_sourceMesh->getNumVertices() == m_targetMesh->getNumVertices()) <<
				"Source and target mesh have differing vertex count, can't use MeshToMeshTransfer.";
		return true;
	}


	std::shared_ptr<SourceType> getSourceMesh()
	{
		return m_sourceMesh;
	}

	void setSourceMesh(std::shared_ptr<SourceType> val)
	{
		m_sourceMesh = val;
	}

	std::shared_ptr<TargetType> getTargetMesh()
	{
		return m_targetMesh;
	}

	void setTargetMesh(std::shared_ptr<TargetType> val)
	{
		m_targetMesh = val;
	}

private:
	std::shared_ptr<SourceType> m_sourceMesh;
	std::shared_ptr<TargetType> m_targetMesh;
};

static std::shared_ptr<SurgSim::DataStructures::TriangleMeshPlyReaderDelegate::MeshType>
loadMesh(const std::string& fileName)
{
	// The PlyReader and TriangleMeshPlyReaderDelegate work together to load triangle meshes.
	SurgSim::DataStructures::PlyReader reader(fileName);
	auto delegate = std::make_shared<SurgSim::DataStructures::TriangleMeshPlyReaderDelegate>();

	SURGSIM_ASSERT(reader.setDelegate(delegate)) << "The input file " << fileName << " is malformed.";
	reader.parseFile();

	return delegate->getMesh();
}

static std::shared_ptr<SurgSim::Framework::SceneElement> createFemSceneElement(
	const std::string& name,
	const std::string& filename,
	SurgSim::Math::IntegrationScheme integrationScheme)
{
	// Create a SceneElement that bundles the pieces associated with the finite element model
	auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>(name);

	// Load the tetrahedral mesh and initialize the finite element model
	auto fem = std::make_shared<SurgSim::Physics::Fem3DRepresentation>("fem3d");
	fem->setFilename(filename);
	fem->setIntegrationScheme(integrationScheme);
	fem->loadFile();
	sceneElement->addComponent(fem);

	// The mesh for visualizing the surface of the finite element model
	auto graphics = std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>("fem graphics");
	*graphics->getMesh() = *(std::make_shared<SurgSim::Graphics::Mesh>(*loadMesh(filename)));
	graphics->setDrawAsWireFrame(true);
	sceneElement->addComponent(graphics);

	// Create a behavior which transfers the position of the vertices in the FEM to locations in the triangle mesh
	auto femToMesh =
		std::make_shared<SurgSim::Blocks::TransferDeformableStateToVerticesBehavior<SurgSim::Graphics::VertexData>>(
			name + " physics to triangle mesh",
			fem->getFinalState(),
			graphics->getMesh());
	sceneElement->addComponent(femToMesh);

	auto collision = std::make_shared<SurgSim::Physics::DeformableCollisionRepresentation>("collision");
	collision->setMesh(std::make_shared<SurgSim::DataStructures::TriangleMesh>(*loadMesh(filename)));
	sceneElement->addComponent(collision);
	fem->setCollisionRepresentation(collision);

	// The mesh for visualizing the collision mesh
	graphics = std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>("collision graphics");
	*graphics->getMesh() = *(std::make_shared<SurgSim::Graphics::Mesh>(*loadMesh(filename)));
	auto mesh = graphics->getMesh();
	for (size_t i = 0; i < mesh->getNumVertices(); ++i)
	{
		mesh->getVertex(i).data.color.setValue(SurgSim::Math::Vector4d(0.8, 0.2, 0.2, 1.0));
	}
	graphics->setDrawAsWireFrame(true);
	sceneElement->addComponent(graphics);

	auto collisionToMesh =
		std::make_shared<MeshToMeshTransfer<SurgSim::DataStructures::TriangleMesh, SurgSim::Graphics::Mesh>>
		("collision to graphics");
	collisionToMesh->setSourceMesh(collision->getMesh());
	collisionToMesh->setTargetMesh(graphics->getMesh());
	sceneElement->addComponent(collisionToMesh);

	// The point-cloud for visualizing the nodes of the finite element model
	auto pointCloud
		= std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<EmptyData>>("point cloud");
	pointCloud->setColor(SurgSim::Math::Vector4d(0.2, 0.2, 1.0, 1.0));
	pointCloud->setPointSize(3.0f);
	pointCloud->setVisible(true);
	sceneElement->addComponent(pointCloud);

	// The behavior which transfers the position of the vertices in the FEM to locations in the point cloud
	auto femToCloud = std::make_shared<SurgSim::Blocks::TransferDeformableStateToVerticesBehavior<EmptyData>>(
						  "fem to point cloud",
						  fem->getFinalState(),
						  pointCloud->getVertices());
	sceneElement->addComponent(femToCloud);


	return sceneElement;
}

TEST_F(RenderTests, MeshRenderTest)
{
	auto data = runtime->getApplicationData();
	auto filename = data->findFile("Fem3DMeshRenderTest/wound_deformable.ply");

	auto fem = createFemSceneElement("Fem", filename, SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER);

	runtime->getScene()->addSceneElement(fem);

	runTest(Vector3d(0.0, 0.0, 0.2), Vector3d::Zero(), 5000.0);
}

}
}

