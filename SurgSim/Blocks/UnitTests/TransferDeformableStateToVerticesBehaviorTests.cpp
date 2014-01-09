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

/// \file
/// Tests for the TransferDeformableStateToVerticesBehavior class.

#include <gtest/gtest.h>

#include "SurgSim/Blocks/BasicSceneElement.h"
#include "SurgSim/Blocks/TransferDeformableStateToVerticesBehavior.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"

using SurgSim::Blocks::BasicSceneElement;
using SurgSim::Blocks::TransferDeformableStateToVerticesBehavior;
using SurgSim::DataStructures::Vertices;
using SurgSim::Physics::DeformableRepresentationState;

namespace
{
class ExtraData
{
public:
	SurgSim::Math::Vector3d v;
	SurgSim::Math::Matrix33d m;
	std::string s;

	bool operator ==(const ExtraData& e) const
	{
		return v == e.v && m == e.m && s == e.s;
	}

	bool operator !=(const ExtraData& e) const
	{
		return ! ((*this) == e);
	}
};
};

template <class T>
void testConstructor()
{
	const unsigned int numDofPerNode = 3;
	const unsigned int numNode = 10;

	std::shared_ptr<DeformableRepresentationState> state;
	state = std::make_shared<DeformableRepresentationState>();
	state->setNumDof(numDofPerNode, numNode);
	state->getPositions().setRandom();

	std::shared_ptr<Vertices<T>> vertices;
	vertices = std::make_shared<Vertices<T>>();

	ASSERT_NO_THROW({TransferDeformableStateToVerticesBehavior<T> m("name", state, vertices);});
	ASSERT_NO_THROW({TransferDeformableStateToVerticesBehavior<T>* m = \
		new TransferDeformableStateToVerticesBehavior<T>("name", state, vertices); delete m;});
	ASSERT_NO_THROW({std::shared_ptr<TransferDeformableStateToVerticesBehavior<T>> m = \
		std::make_shared<TransferDeformableStateToVerticesBehavior<T>>("name", state, vertices);});
}

TEST(TransferDeformableStateToVerticesBehaviorTests, ConstructorTest)
{
	{
		SCOPED_TRACE("'void' template parameter");
		testConstructor<void>();
	}
	{
		SCOPED_TRACE("'int' template parameter");
		testConstructor<int>();
	}
	{
		SCOPED_TRACE("'double' template parameter");
		testConstructor<double>();
	}
	{
		SCOPED_TRACE("'Vector3d' template parameter");
		testConstructor<SurgSim::Math::Vector3d>();
	}
	{
		SCOPED_TRACE("'Matrix33d' template parameter");
		testConstructor<SurgSim::Math::Matrix33d>();
	}
	{
		SCOPED_TRACE("'class ExtraData' template parameter");
		testConstructor<ExtraData>();
	}
}

template <class T>
void testUpdate()
{
	using SurgSim::Math::getSubVector;

	const unsigned int numDofPerNode = 3;
	const unsigned int numNode = 10;

	std::shared_ptr<SurgSim::Framework::Runtime> runtime = std::make_shared<SurgSim::Framework::Runtime>();
	std::shared_ptr<SurgSim::Framework::BehaviorManager> behaviorManager =
		std::make_shared<SurgSim::Framework::BehaviorManager>();
	runtime->addManager(behaviorManager);

	/// Fetch the scene from the runtime
	std::shared_ptr<SurgSim::Framework::Scene> scene = runtime->getScene();
	/// Add the representations and behavior to a scene element
	std::shared_ptr<BasicSceneElement> sceneElement = std::make_shared<BasicSceneElement>("scene element");
	std::shared_ptr<TransferDeformableStateToVerticesBehavior<T>> m;
	std::shared_ptr<Vertices<T>> vertices;
	std::shared_ptr<DeformableRepresentationState> state;
	{
		state = std::make_shared<DeformableRepresentationState>();
		state->setNumDof(numDofPerNode, numNode);
		state->getPositions().setRandom();

		vertices = std::make_shared<Vertices<T>>();

		m = std::make_shared<TransferDeformableStateToVerticesBehavior<T>>("Transfer Behavior", state, vertices);
	}
	sceneElement->addComponent(m);
	scene->addSceneElement(sceneElement);

	EXPECT_FALSE(behaviorManager->isInitialized());
	EXPECT_FALSE(behaviorManager->isRunning());

	// This will initialize the data structure properly and wake the threads up
	runtime->start();
	// Give some time to the thread to start and initialize internal data (useful notably for the isRunning() call)
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));

	EXPECT_TRUE(behaviorManager->isInitialized());
	EXPECT_TRUE(behaviorManager->isRunning());

	m->update(1.0);
	ASSERT_EQ(numNode, vertices->getNumVertices());
	for (unsigned int nodeId = 0; nodeId < numNode; nodeId++)
	{
		ASSERT_EQ(getSubVector(state->getPositions(), nodeId, numDofPerNode), vertices->getVertexPosition(nodeId));
		//ASSERT_EQ(T(), vertices->getVertex(nodeId).data);
		// Cannot test the data component has it is created in the behavior on 1st pass, using the default constructor.
		// But some classes only allocate the memory but does not reset it (Vector3d, Matrix33d,...), so the content
		// is not well defined and therefore cannot be tested with a newly created instance here.
	}
}

template <>
void testUpdate<void>()
{
	using SurgSim::Math::getSubVector;

	const unsigned int numDofPerNode = 3;
	const unsigned int numNode = 10;

	std::shared_ptr<SurgSim::Framework::Runtime> runtime = std::make_shared<SurgSim::Framework::Runtime>();
	std::shared_ptr<SurgSim::Framework::BehaviorManager> behaviorManager =
		std::make_shared<SurgSim::Framework::BehaviorManager>();
	runtime->addManager(behaviorManager);

	/// Fetch the scene and add the scene element to it
	std::shared_ptr<SurgSim::Framework::Scene> scene = runtime->getScene();
	/// Add the representations and behavior to a scene element
	std::shared_ptr<BasicSceneElement> sceneElement = std::make_shared<BasicSceneElement>("scene element");
	std::shared_ptr<TransferDeformableStateToVerticesBehavior<void>> m;
	std::shared_ptr<Vertices<void>> vertices;
	std::shared_ptr<DeformableRepresentationState> state;
	{
		state = std::make_shared<DeformableRepresentationState>();
		state->setNumDof(numDofPerNode, numNode);
		state->getPositions().setRandom();

		vertices = std::make_shared<Vertices<void>>();

		m = std::make_shared<TransferDeformableStateToVerticesBehavior<void>>("Transfer Behavior", state, vertices);
	}
	sceneElement->addComponent(m);
	scene->addSceneElement(sceneElement);

	EXPECT_FALSE(behaviorManager->isInitialized());
	EXPECT_FALSE(behaviorManager->isRunning());

	// This will initialize the data structure properly and wake the threads up
	runtime->start();
	// Give some time to the thread to start and initialize internal data (useful notably for the isRunning() call)
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));

	EXPECT_TRUE(behaviorManager->isInitialized());
	EXPECT_TRUE(behaviorManager->isRunning());

	m->update(1.0);
	ASSERT_EQ(numNode, vertices->getNumVertices());
	for (unsigned int nodeId = 0; nodeId < numNode; nodeId++)
	{
		ASSERT_EQ(getSubVector(state->getPositions(), nodeId, numDofPerNode), vertices->getVertexPosition(nodeId));
	}
}

TEST(TransferDeformableStateToVerticesBehaviorTests, UpdateTest)
{
	{
		SCOPED_TRACE("'void' template parameter");
		testUpdate<void>();
	}
	{
		SCOPED_TRACE("'int' template parameter");
		testUpdate<int>();
	}
	{
		SCOPED_TRACE("'double' template parameter");
		testUpdate<double>();
	}
	{
		SCOPED_TRACE("'Vector3d' template parameter");
		testUpdate<SurgSim::Math::Vector3d>();
	}
	{
		SCOPED_TRACE("'Matrix33d' template parameter");
		testUpdate<SurgSim::Math::Matrix33d>();
	}
	{
		SCOPED_TRACE("'class ExtraData' template parameter");
		testUpdate<ExtraData>();
	}
}
