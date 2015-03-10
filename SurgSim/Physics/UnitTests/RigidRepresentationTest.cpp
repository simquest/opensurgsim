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

#include <string>

#include <gtest/gtest.h>

#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/DataStructures/BufferedValue.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/PlaneShape.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Valid.h"
#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"

using SurgSim::DataStructures::Location;
using SurgSim::Framework::Component;
using SurgSim::Framework::Runtime;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::Matrix66d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::PlaneShape;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Shape;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector6d;

namespace
{
const double epsilon = 1e-10;
}

namespace SurgSim
{
namespace Physics
{

class RigidRepresentationTest : public ::testing::Test
{
public:
	void SetUp()
	{
		m_dt = 1e-3;

		m_radius = 0.1;
		m_density = 9000.0;
		m_mass = 4.0 / 3.0 * M_PI * m_radius * m_radius * m_radius * m_density;
		double coef = 2.0 / 5.0 * m_mass * m_radius * m_radius;
		m_inertia << coef, 0.0, 0.0, 0.0, coef, 0.0, 0.0, 0.0, coef;
		m_id33.setIdentity();
		m_zero33.setZero();
		m_invalidInertia.setRandom();
		m_invalidInertia = m_invalidInertia + m_invalidInertia.transpose().eval(); // make symmetric
		m_invalidInertia(0, 0) = -12.3; // Negative value on hte diagonal (invalid)
		m_sphere = std::make_shared<SphereShape>(m_radius);

		Quaterniond q(0.5, 0.4, 0.3, 0.2);
		q.normalize();
		Vector3d t(1.2, 2.1, 12.21);
		m_state.setAngularVelocity(Vector3d(1, 2, 3));
		m_state.setLinearVelocity(Vector3d(3, 2, 1));
		m_state.setPose(SurgSim::Math::makeRigidTransform(q, t));

		m_maxNumSimulationStepTest = 100;
	}

	void TearDown()
	{
	}

	// Time step
	double m_dt;

	// Sphere radius (in m)
	double m_radius;

	// Sphere density (in Kg.m-3)
	double m_density;

	// Sphere mass (in Kg)
	double m_mass;

	// Sphere inertia matrix
	SurgSim::Math::Matrix33d m_inertia;

	// Identity matrix 3x3 (for convenience)
	SurgSim::Math::Matrix33d m_id33;

	// Zero matrix 3x3 (for convenience)
	SurgSim::Math::Matrix33d m_zero33;

	// Invalid inertia matrix
	SurgSim::Math::Matrix33d m_invalidInertia;

	// SphereShape
	std::shared_ptr<SphereShape> m_sphere;

	// Rigid representation state
	RigidRepresentationState m_state;

	// Rigid representation default state
	RigidRepresentationState m_defaultState;

	// Max number of simulation step for testing
	int m_maxNumSimulationStepTest;
};

TEST_F(RigidRepresentationTest, ConstructorTest)
{
	ASSERT_NO_THROW(RigidRepresentation rigidBody("Rigid"));
}

TEST_F(RigidRepresentationTest, ResetTest)
{
	// Create the rigid body
	std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");

	rigidBody->setInitialState(m_state);
	rigidBody->setLocalActive(false);
	rigidBody->setIsGravityEnabled(false);
	rigidBody->setLocalPose(RigidTransform3d::Identity());

	// reset the representation state
	rigidBody->resetState();

	// isActive unchanged
	EXPECT_FALSE(rigidBody->isActive());
	// isGravityEnable flag unchanged
	EXPECT_FALSE(rigidBody->isGravityEnabled());
	// current state = initial state
	EXPECT_TRUE(rigidBody->getInitialState() == rigidBody->getCurrentState());
	// previous state = initial state
	EXPECT_TRUE(rigidBody->getInitialState() == rigidBody->getPreviousState());
}

TEST_F(RigidRepresentationTest, SetGetAndDefaultValueTest)
{
	// Create the rigid body
	std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");

	// Get state (current, initial)
	EXPECT_TRUE(m_defaultState == rigidBody->getCurrentState());
	EXPECT_TRUE(m_defaultState == rigidBody->getPreviousState());
	EXPECT_TRUE(m_defaultState == rigidBody->getInitialState());
	rigidBody->setInitialState(m_state);
	EXPECT_TRUE(m_state == rigidBody->getInitialState());
	EXPECT_TRUE(m_state == rigidBody->getCurrentState());
	EXPECT_TRUE(m_state == rigidBody->getPreviousState());

	// Mass density [default = 0]
	EXPECT_NEAR(0.0, rigidBody->getDensity(), epsilon);
	// Mass [default = qNaA]
	EXPECT_FALSE(SurgSim::Math::isValid(rigidBody->getMass()));
	// Inertia 3x3 symmetric matrix [default = qNaN values]
	EXPECT_FALSE(SurgSim::Math::isValid(rigidBody->getLocalInertia()));
	// Linear damping [default = 0]
	EXPECT_NEAR(0.0, rigidBody->getLinearDamping(), epsilon);
	// Angular damping [default = 0]
	EXPECT_NEAR(0.0, rigidBody->getAngularDamping(), epsilon);
	// Shape [default = nullptr]
	EXPECT_EQ(nullptr, rigidBody->getShape());

	// Mass density
	rigidBody->setDensity(m_density);
	EXPECT_NEAR(m_density, rigidBody->getDensity(), epsilon);
	rigidBody->setDensity(0.0);
	EXPECT_NEAR(0.0, rigidBody->getDensity(), epsilon);

	// Linear damping
	rigidBody->setLinearDamping(5.5);
	EXPECT_NEAR(5.5, rigidBody->getLinearDamping(), epsilon);
	rigidBody->setLinearDamping(0.0);
	EXPECT_NEAR(0.0, rigidBody->getLinearDamping(), epsilon);

	// Angular damping
	rigidBody->setAngularDamping(5.5);
	EXPECT_NEAR(5.5, rigidBody->getAngularDamping(), epsilon);
	rigidBody->setAngularDamping(0.0);
	EXPECT_NEAR(0.0, rigidBody->getAngularDamping(), epsilon);

	// Shape
	rigidBody->setShape(m_sphere);
	EXPECT_EQ(m_sphere, rigidBody->getShape());
	rigidBody->setShape(nullptr);
	EXPECT_EQ(nullptr, rigidBody->getShape());

	// Get/Set active flag [default = true]
	EXPECT_TRUE(rigidBody->isActive());
	EXPECT_TRUE(rigidBody->isLocalActive());
	rigidBody->setLocalActive(false);
	ASSERT_FALSE(rigidBody->isActive());
	ASSERT_FALSE(rigidBody->isLocalActive());
	rigidBody->setLocalActive(true);
	ASSERT_TRUE(rigidBody->isActive());
	ASSERT_TRUE(rigidBody->isLocalActive());

	// Get numDof = 6
	ASSERT_EQ(6u, rigidBody->getNumDof());

	// Set/Get isGravityEnabled [default = true]
	EXPECT_TRUE(rigidBody->isGravityEnabled());
	rigidBody->setIsGravityEnabled(false);
	ASSERT_FALSE(rigidBody->isGravityEnabled());
	rigidBody->setIsGravityEnabled(true);
	ASSERT_TRUE(rigidBody->isGravityEnabled());
}

TEST_F(RigidRepresentationTest, AddExternalGeneralizedForceOnMassCenterTest)
{
	std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
	rigidBody->setShape(m_sphere);

	Vector6d F = 6.4 * Vector6d::Ones();
	Matrix66d K = 0.4 * Matrix66d::Ones() + 8.3 * Matrix66d::Identity();
	Matrix66d D = -0.6 * Matrix66d::Ones() + 4.1 * Matrix66d::Identity();

	rigidBody->addExternalGeneralizedForce(F, K, D);
	EXPECT_TRUE(rigidBody->getExternalGeneralizedForce().unsafeGet().isApprox(F));
	EXPECT_TRUE(rigidBody->getExternalGeneralizedStiffness().isApprox(K));
	EXPECT_TRUE(rigidBody->getExternalGeneralizedDamping().isApprox(D));
}

namespace
{
// Extra force/torque that should be added
Vector6d computeExtraTorque(const Vector6d& inputForce, const Vector3d& anchorLocalPoint,
							const Vector6d& dofPosition, const Vector6d& dofVelocity)
{
	Vector6d f = inputForce;

	auto C = dofPosition.segment<3>(0);               // Mass center
	auto rotVector = dofPosition.segment<3>(3);       // Rotation vector

	double angle;
	Vector3d axis = rotVector;
	angle = axis.norm();
	if (std::abs(angle) < 1e-8)
	{
		axis.setZero();
	}
	else
	{
		axis.normalize();
	}
	Matrix33d R = SurgSim::Math::makeRotationMatrix(angle, axis);

	Vector3d anchorPoint = C + R * anchorLocalPoint;
	Vector3d lever = anchorPoint - C;

	f.segment<3>(3) += lever.cross(f.segment<3>(0));

	return f;
}

Matrix66d computeExtraStiffness(const Vector6d& inputForce, const Vector3d& anchorLocalPoint,
								const Vector6d& dofPosition, const Vector6d& dofVelocity)
{
	Matrix66d K = Matrix66d::Zero();
	const double epsilon = 1e-8;

	for (size_t column = 0; column < 6; ++column)
	{
		Vector6d dofX = dofPosition;
		dofX[column] += 2.0 * epsilon;
		Vector6d fXPlus2H = computeExtraTorque(inputForce, anchorLocalPoint, dofX, dofVelocity);

		dofX = dofPosition;
		dofX[column] += epsilon;
		Vector6d fXPlusH = computeExtraTorque(inputForce, anchorLocalPoint, dofX, dofVelocity);

		dofX = dofPosition;
		dofX[column] -= epsilon;
		Vector6d fXMinusH = computeExtraTorque(inputForce, anchorLocalPoint, dofX, dofVelocity);

		dofX = dofPosition;
		dofX[column] -= 2.0 * epsilon;
		Vector6d fXMinus2H = computeExtraTorque(inputForce, anchorLocalPoint, dofX, dofVelocity);

		K.col(column) = - (-fXPlus2H + 8.0 * fXPlusH - 8.0 * fXMinusH + fXMinus2H) / (12.0 * epsilon);
	}

	return K;
}

Matrix66d computeExtraDamping(const Vector6d& inputForce, const Vector3d& anchorLocalPoint,
							  const Vector6d& dofPosition, const Vector6d& dofVelocity)
{
	Matrix66d D = Matrix66d::Zero();
	const double epsilon = 1e-8;

	for (size_t column = 0; column < 6; ++column)
	{
		Vector6d dofV = dofVelocity;
		dofV[column] += 2.0 * epsilon;
		Vector6d fXPlus2H = computeExtraTorque(inputForce, anchorLocalPoint, dofPosition, dofV);

		dofV = dofVelocity;
		dofV[column] += epsilon;
		Vector6d fXPlusH = computeExtraTorque(inputForce, anchorLocalPoint, dofPosition, dofV);

		dofV = dofVelocity;
		dofV[column] -= epsilon;
		Vector6d fXMinusH = computeExtraTorque(inputForce, anchorLocalPoint, dofPosition, dofV);

		dofV = dofVelocity;
		dofV[column] -= 2.0 * epsilon;
		Vector6d fXMinus2H = computeExtraTorque(inputForce, anchorLocalPoint, dofPosition, dofV);

		D.col(column) = - (-fXPlus2H + 8.0 * fXPlusH - 8.0 * fXMinusH + fXMinus2H) / (12.0 * epsilon);
	}

	return D;
}
}; // namespace anonymous

TEST_F(RigidRepresentationTest, AddExternalGeneralizedForceExtraTermsTest)
{
	{
		SCOPED_TRACE("Non identity pose");

		Eigen::AngleAxisd angleAxis(0.34512, Vector3d(1.1, -1.4, 3.23).normalized());
		Vector3d t(1.1, 2.2, 3.3);
		RigidTransform3d transform = makeRigidTransform(Quaterniond(angleAxis), t);

		Vector6d inputForce = Vector6d::LinSpaced(1.1, 6.6);
		Vector6d dofX = Vector6d::Zero();
		dofX.segment<3>(0) = transform.translation();
		Eigen::AngleAxisd angleAxis2(transform.rotation());
		dofX.segment<3>(3) = angleAxis2.axis() * angleAxis2.angle();
		Vector6d dofV = Vector6d::Zero();
		Vector3d anchorLocalPoint = Vector3d::Ones();

		std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
		rigidBody->setShape(m_sphere);
		SurgSim::Physics::RigidRepresentationState initialState;
		initialState.setPose(transform);
		rigidBody->setInitialState(initialState);
		SurgSim::DataStructures::Location location(anchorLocalPoint);

		Vector6d Fnumeric = computeExtraTorque(inputForce, anchorLocalPoint, dofX, dofV);
		Matrix66d Knumeric = computeExtraStiffness(inputForce, anchorLocalPoint, dofX, dofV);
		Matrix66d Dnumeric = computeExtraDamping(inputForce, anchorLocalPoint, dofX, dofV);

		Vector6d F = inputForce;
		Matrix66d K = Matrix66d::Zero(), D = Matrix66d::Zero();
		rigidBody->addExternalGeneralizedForce(location, F, K, D);
		F = rigidBody->getExternalGeneralizedForce().unsafeGet();
		K = rigidBody->getExternalGeneralizedStiffness();
		D = rigidBody->getExternalGeneralizedDamping();

		EXPECT_LE((F - Fnumeric).cwiseAbs().maxCoeff(), 2e-7);
		EXPECT_LE((K - Knumeric).cwiseAbs().maxCoeff(), 2e-7);
		EXPECT_LE((D - Dnumeric).cwiseAbs().maxCoeff(), 2e-7);
	}

	{
		SCOPED_TRACE("Exactly Identity pose");

		Vector6d inputForce = Vector6d::LinSpaced(1.1, 6.6);
		Vector6d dofX = Vector6d::Zero();
		Vector6d dofV = Vector6d::Zero();
		Vector3d anchorLocalPoint = Vector3d::Ones();

		std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
		rigidBody->setShape(m_sphere);
		SurgSim::DataStructures::Location location(anchorLocalPoint);

		Vector6d Fnumeric = computeExtraTorque(inputForce, anchorLocalPoint, dofX, dofV);
		Matrix66d Knumeric = computeExtraStiffness(inputForce, anchorLocalPoint, dofX, dofV);
		Matrix66d Dnumeric = computeExtraDamping(inputForce, anchorLocalPoint, dofX, dofV);

		Vector6d F = inputForce;
		Matrix66d K = Matrix66d::Zero(), D = Matrix66d::Zero();
		rigidBody->addExternalGeneralizedForce(location, F, K, D);
		F = rigidBody->getExternalGeneralizedForce().unsafeGet();
		K = rigidBody->getExternalGeneralizedStiffness();
		D = rigidBody->getExternalGeneralizedDamping();

		EXPECT_LE((F - Fnumeric).cwiseAbs().maxCoeff(), 2e-7);
		EXPECT_LE((K - Knumeric).cwiseAbs().maxCoeff(), 2e-7);
		EXPECT_LE((D - Dnumeric).cwiseAbs().maxCoeff(), 2e-7);
	}

	{
		SCOPED_TRACE("Almost Identity pose, limitted development not used yet");

		Eigen::AngleAxisd angleAxis(5e-8, Vector3d(1.1, -1.4, 3.23).normalized());
		Vector3d t(1.1, 2.2, 3.3);
		RigidTransform3d transform = makeRigidTransform(Quaterniond(angleAxis), t);

		Vector6d inputForce = Vector6d::LinSpaced(1.1, 6.6);
		Vector6d dofX = Vector6d::Zero();
		dofX.segment<3>(0) = transform.translation();
		Eigen::AngleAxisd angleAxis2(transform.rotation());
		dofX.segment<3>(3) = angleAxis2.angle() * angleAxis2.axis();
		Vector6d dofV = Vector6d::Zero();
		Vector3d anchorLocalPoint = Vector3d::Ones();

		std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
		rigidBody->setShape(m_sphere);
		SurgSim::Physics::RigidRepresentationState initialState;
		initialState.setPose(transform);
		rigidBody->setInitialState(initialState);
		SurgSim::DataStructures::Location location(anchorLocalPoint);

		Vector6d Fnumeric = computeExtraTorque(inputForce, anchorLocalPoint, dofX, dofV);
		Matrix66d Knumeric = computeExtraStiffness(inputForce, anchorLocalPoint, dofX, dofV);
		std::cout << "Knumeric:" << std::endl << Knumeric << std::endl;
		Matrix66d Dnumeric = computeExtraDamping(inputForce, anchorLocalPoint, dofX, dofV);
		Vector6d F = inputForce;
		Matrix66d K = Matrix66d::Zero(), D = Matrix66d::Zero();
		rigidBody->addExternalGeneralizedForce(location, F, K, D);
		F = rigidBody->getExternalGeneralizedForce().unsafeGet();
		K = rigidBody->getExternalGeneralizedStiffness();
		D = rigidBody->getExternalGeneralizedDamping();
		std::cout << "K:" << std::endl << K << std::endl << "Knumeric:" <<
				  std::endl << Knumeric << std::endl;

		EXPECT_LE((F - Fnumeric).cwiseAbs().maxCoeff(), 2e-7);
		EXPECT_LE((K - Knumeric).cwiseAbs().maxCoeff(), 2.2e-7); // Epsilon set by trial and error
		EXPECT_LE((D - Dnumeric).cwiseAbs().maxCoeff(), 2e-7);
	}

	{
		SCOPED_TRACE("Almost Identity pose, limitted development in use");

		Eigen::AngleAxisd angleAxis(0.2e-8, Vector3d(1.1, -1.4, 3.23).normalized());
		Vector3d t(1.1, 2.2, 3.3);
		RigidTransform3d transform = makeRigidTransform(Quaterniond(angleAxis), t);

		Vector6d inputForce = Vector6d::LinSpaced(1.1, 6.6);
		Vector6d dofX = Vector6d::Zero();
		dofX.segment<3>(0) = transform.translation();
		Eigen::AngleAxisd angleAxis2(transform.rotation());
		dofX.segment<3>(3) = angleAxis2.angle() * angleAxis2.axis();
		Vector6d dofV = Vector6d::Zero();
		Vector3d anchorLocalPoint = Vector3d::Ones();

		std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
		rigidBody->setShape(m_sphere);
		SurgSim::Physics::RigidRepresentationState initialState;
		initialState.setPose(transform);
		rigidBody->setInitialState(initialState);
		SurgSim::DataStructures::Location location(anchorLocalPoint);

		Vector6d Fnumeric = computeExtraTorque(inputForce, anchorLocalPoint, dofX, dofV);
		Matrix66d Knumeric = computeExtraStiffness(inputForce, anchorLocalPoint, dofX, dofV);
		Matrix66d Dnumeric = computeExtraDamping(inputForce, anchorLocalPoint, dofX, dofV);

		Vector6d F = inputForce;
		Matrix66d K = Matrix66d::Zero(), D = Matrix66d::Zero();
		rigidBody->addExternalGeneralizedForce(location, F, K, D);
		F = rigidBody->getExternalGeneralizedForce().unsafeGet();
		K = rigidBody->getExternalGeneralizedStiffness();
		D = rigidBody->getExternalGeneralizedDamping();

		EXPECT_LE((F - Fnumeric).cwiseAbs().maxCoeff(), 2e-7);
		EXPECT_LE((K - Knumeric).cwiseAbs().maxCoeff(), 2e-7);
		EXPECT_LE((D - Dnumeric).cwiseAbs().maxCoeff(), 2e-7);
	}
}

namespace
{
Vector6d computeSpringForce(double linearStiffness, double linearDamping,
							double angularStiffness, double angularDamping,
							Vector3d targetPosition, Vector3d targetLinearVelocity,
							Vector3d targetRotationVector, Vector3d targetAngularVelocity,
							Vector6d dofX, Vector6d dofV,
							bool doComputeExtraTorque = false, Vector3d localApplicationPoint = Vector3d::Zero())
{
	Vector6d f = Vector6d::Zero();

	f.segment<3>(0) += linearStiffness * (targetPosition - dofX.segment<3>(0));
	f.segment<3>(0) += linearDamping * (targetLinearVelocity - dofV.segment<3>(0));

	f.segment<3>(3) += angularStiffness * (targetRotationVector - dofX.segment<3>(3));
	f.segment<3>(3) += angularDamping * (targetAngularVelocity - dofV.segment<3>(3));
	if (doComputeExtraTorque)
	{
		Vector3d rotationVector = dofX.segment<3>(3);
		Matrix33d R;
		double angle = rotationVector.norm();
		if (std::abs(angle) < 1e-8)
		{
			R.setIdentity();
		}
		else
		{
			Vector3d axis = rotationVector / angle;
			R = cos(angle) * Matrix33d::Identity() +
				sin(angle) * SurgSim::Math::makeSkewSymmetricMatrix(axis) +
				(1.0 - cos(angle)) * axis * axis.transpose();
		}
		Vector3d applicationPoint = dofX.segment<3>(0) + R * localApplicationPoint;
		Vector3d lever = applicationPoint - dofX.segment<3>(0);
		f.segment<3>(3) += lever.cross(f.segment<3>(0));
	}

	return f;
}

Matrix66d computeSpringStiffness(double linearStiffness, double linearDamping,
								 double angularStiffness, double angularDamping,
								 Vector3d targetPosition, Vector3d targetLinearVelocity,
								 Vector3d targetRotationVector, Vector3d targetAngularVelocity,
								 Vector6d dofPosition, Vector6d dofVelocity,
								 bool doComputeExtraTorque = false, Vector3d localApplicationPoint = Vector3d::Zero())
{
	Matrix66d K = Matrix66d::Zero();
	const double epsilon = 1e-8;

	for (size_t column = 0; column < 6; ++column)
	{
		Vector6d dofX = dofPosition;
		dofX[column] += 2.0 * epsilon;
		Vector6d fXPlus2H = computeSpringForce(linearStiffness, linearDamping, angularStiffness, angularDamping,
											   targetPosition, targetLinearVelocity, targetRotationVector,
											   targetAngularVelocity, dofX, dofVelocity,
											   doComputeExtraTorque, localApplicationPoint);

		dofX = dofPosition;
		dofX[column] += epsilon;
		Vector6d fXPlusH = computeSpringForce(linearStiffness, linearDamping, angularStiffness, angularDamping,
											  targetPosition, targetLinearVelocity, targetRotationVector,
											  targetAngularVelocity, dofX, dofVelocity,
											  doComputeExtraTorque, localApplicationPoint);

		dofX = dofPosition;
		dofX[column] -= epsilon;
		Vector6d fXMinusH = computeSpringForce(linearStiffness, linearDamping, angularStiffness, angularDamping,
											   targetPosition, targetLinearVelocity, targetRotationVector,
											   targetAngularVelocity, dofX, dofVelocity,
											   doComputeExtraTorque, localApplicationPoint);

		dofX = dofPosition;
		dofX[column] -= 2.0 * epsilon;
		Vector6d fXMinus2H = computeSpringForce(linearStiffness, linearDamping, angularStiffness, angularDamping,
												targetPosition, targetLinearVelocity, targetRotationVector,
												targetAngularVelocity, dofX, dofVelocity,
												doComputeExtraTorque, localApplicationPoint);

		K.col(column) = - (-fXPlus2H + 8.0 * fXPlusH - 8.0 * fXMinusH + fXMinus2H) / (12.0 * epsilon);
	}

	return K;
}

Matrix66d computeSpringDamping(double linearStiffness, double linearDamping,
							   double angularStiffness, double angularDamping,
							   Vector3d targetPosition, Vector3d targetLinearVelocity,
							   Vector3d targetRotationVector, Vector3d targetAngularVelocity,
							   Vector6d dofPosition, Vector6d dofVelocity,
							   bool doComputeExtraTorque = false, Vector3d localApplicationPoint = Vector3d::Zero())
{
	Matrix66d D = Matrix66d::Zero();
	const double epsilon = 1e-8;

	for (size_t column = 0; column < 6; ++column)
	{
		Vector6d dofV = dofVelocity;
		dofV[column] += 2.0 * epsilon;
		Vector6d fXPlus2H = computeSpringForce(linearStiffness, linearDamping, angularStiffness, angularDamping,
											   targetPosition, targetLinearVelocity, targetRotationVector,
											   targetAngularVelocity, dofPosition, dofV,
											   doComputeExtraTorque, localApplicationPoint);

		dofV = dofVelocity;
		dofV[column] += epsilon;
		Vector6d fXPlusH = computeSpringForce(linearStiffness, linearDamping, angularStiffness, angularDamping,
											  targetPosition, targetLinearVelocity, targetRotationVector,
											  targetAngularVelocity, dofPosition, dofV,
											  doComputeExtraTorque, localApplicationPoint);

		dofV = dofVelocity;
		dofV[column] -= epsilon;
		Vector6d fXMinusH = computeSpringForce(linearStiffness, linearDamping, angularStiffness, angularDamping,
											   targetPosition, targetLinearVelocity, targetRotationVector,
											   targetAngularVelocity, dofPosition, dofV,
											   doComputeExtraTorque, localApplicationPoint);

		dofV = dofVelocity;
		dofV[column] -= 2.0 * epsilon;
		Vector6d fXMinus2H = computeSpringForce(linearStiffness, linearDamping, angularStiffness, angularDamping,
												targetPosition, targetLinearVelocity, targetRotationVector,
												targetAngularVelocity, dofPosition, dofV,
												doComputeExtraTorque, localApplicationPoint);

		D.col(column) = - (-fXPlus2H + 8.0 * fXPlusH - 8.0 * fXMinusH + fXMinus2H) / (12.0 * epsilon);
	}

	return D;
}
}; // namespace anonymous

TEST_F(RigidRepresentationTest, AddExternalGeneralizedForceTest)
{
	double linearStiffness = 1.1;
	double linearDamping = 1.2;
	double angularStiffness = 1.3;
	double angularDamping = 1.4;
	Vector3d targetPosition = Vector3d::Ones();
	Vector3d targetLinearVelocity = Vector3d::Ones() / 100.0;
	double angle = 1.2354;
	Vector3d axis = Vector3d(1.2, 0.4, 5.2).normalized();
	Vector3d targetRotationVector = axis * angle;
	Vector3d targetAngularVelocity = Vector3d::Ones() / 200.0;
	Vector3d localAnchorPoint = Vector3d::Ones();

	{
		SCOPED_TRACE("With stiffness, with damping");

		Eigen::AngleAxisd angleAxis(0.34512, Vector3d(1.1, -1.4, 3.23).normalized());
		Vector3d t(1.1, 0.9, 1.02);
		RigidTransform3d transform = makeRigidTransform(Quaterniond(angleAxis), t);

		Vector6d dofX = Vector6d::Zero();
		dofX.segment<3>(0) = transform.translation();
		Eigen::AngleAxisd angleAxis2(transform.rotation());
		dofX.segment<3>(3) = angleAxis2.axis() * angleAxis2.angle();
		Vector6d dofV = Vector6d::Zero();

		Vector6d FWithoutExtraTorque = computeSpringForce(linearStiffness, linearDamping,
									   angularStiffness, angularDamping, targetPosition, targetLinearVelocity,
									   targetRotationVector, targetAngularVelocity, dofX, dofV);
		Matrix66d KWithoutExtraTorque = computeSpringStiffness(linearStiffness, linearDamping,
										angularStiffness, angularDamping, targetPosition, targetLinearVelocity,
										targetRotationVector, targetAngularVelocity, dofX, dofV);
		Matrix66d DWithoutExtraTorque = computeSpringDamping(linearStiffness, linearDamping,
										angularStiffness, angularDamping, targetPosition, targetLinearVelocity,
										targetRotationVector, targetAngularVelocity, dofX, dofV);

		Vector6d FWithExtraTorque = computeSpringForce(linearStiffness, linearDamping,
									angularStiffness, angularDamping, targetPosition, targetLinearVelocity,
									targetRotationVector, targetAngularVelocity, dofX, dofV, true, localAnchorPoint);
		Matrix66d KWithExtraTorque = computeSpringStiffness(linearStiffness, linearDamping,
									 angularStiffness, angularDamping, targetPosition, targetLinearVelocity,
									 targetRotationVector, targetAngularVelocity, dofX, dofV, true, localAnchorPoint);
		Matrix66d DWithExtraTorque = computeSpringDamping(linearStiffness, linearDamping,
									 angularStiffness, angularDamping, targetPosition, targetLinearVelocity,
									 targetRotationVector, targetAngularVelocity, dofX, dofV, true, localAnchorPoint);

		std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
		rigidBody->setShape(m_sphere);
		SurgSim::Physics::RigidRepresentationState initialState;
		initialState.setPose(transform);
		rigidBody->setInitialState(initialState);
		SurgSim::DataStructures::Location location(localAnchorPoint);

		Vector6d F = FWithoutExtraTorque;
		Matrix66d K = KWithoutExtraTorque, D = DWithoutExtraTorque;
		rigidBody->addExternalGeneralizedForce(location, F, K, D);
		F = rigidBody->getExternalGeneralizedForce().unsafeGet();
		K = rigidBody->getExternalGeneralizedStiffness();
		D = rigidBody->getExternalGeneralizedDamping();

		EXPECT_LE((F - FWithExtraTorque).cwiseAbs().maxCoeff(), 1e-7);
		EXPECT_LE((K - KWithExtraTorque).cwiseAbs().maxCoeff(), 1e-7);
		EXPECT_LE((D - DWithExtraTorque).cwiseAbs().maxCoeff(), 1e-7);
	}

	{
		SCOPED_TRACE("With stiffness, without damping");

		Eigen::AngleAxisd angleAxis(0.34512, Vector3d(1.1, -1.4, 3.23).normalized());
		Vector3d t(1.1, 0.9, 1.02);
		RigidTransform3d transform = makeRigidTransform(Quaterniond(angleAxis), t);

		Vector6d dofX = Vector6d::Zero();
		dofX.segment<3>(0) = transform.translation();
		Eigen::AngleAxisd angleAxis2(transform.rotation());
		dofX.segment<3>(3) = angleAxis2.axis() * angleAxis2.angle();
		Vector6d dofV = Vector6d::Zero();

		Vector6d FWithoutExtraTorque = computeSpringForce(linearStiffness, linearDamping,
									   angularStiffness, angularDamping, targetPosition, targetLinearVelocity,
									   targetRotationVector, targetAngularVelocity, dofX, dofV);
		Matrix66d KWithoutExtraTorque = computeSpringStiffness(linearStiffness, linearDamping,
										angularStiffness, angularDamping, targetPosition, targetLinearVelocity,
										targetRotationVector, targetAngularVelocity, dofX, dofV);

		Vector6d FWithExtraTorque = computeSpringForce(linearStiffness, linearDamping,
									angularStiffness, angularDamping, targetPosition, targetLinearVelocity,
									targetRotationVector, targetAngularVelocity, dofX, dofV, true, localAnchorPoint);
		Matrix66d KWithExtraTorque = computeSpringStiffness(linearStiffness, linearDamping,
									 angularStiffness, angularDamping, targetPosition, targetLinearVelocity,
									 targetRotationVector, targetAngularVelocity, dofX, dofV, true, localAnchorPoint);

		std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
		rigidBody->setShape(m_sphere);
		SurgSim::Physics::RigidRepresentationState initialState;
		initialState.setPose(transform);
		rigidBody->setInitialState(initialState);
		SurgSim::DataStructures::Location location(localAnchorPoint);

		Vector6d F = FWithoutExtraTorque;
		Matrix66d K = KWithoutExtraTorque;
		rigidBody->addExternalGeneralizedForce(location, F, K);
		F = rigidBody->getExternalGeneralizedForce().unsafeGet();
		K = rigidBody->getExternalGeneralizedStiffness();

		EXPECT_LE((F - FWithExtraTorque).cwiseAbs().maxCoeff(), 1e-7);
		EXPECT_LE((K - KWithExtraTorque).cwiseAbs().maxCoeff(), 1e-7);
	}

	{
		SCOPED_TRACE("Without stiffness, without damping");

		Eigen::AngleAxisd angleAxis(0.34512, Vector3d(1.1, -1.4, 3.23).normalized());
		Vector3d t(1.1, 0.9, 1.02);
		RigidTransform3d transform = makeRigidTransform(Quaterniond(angleAxis), t);

		Vector6d dofX = Vector6d::Zero();
		dofX.segment<3>(0) = transform.translation();
		Eigen::AngleAxisd angleAxis2(transform.rotation());
		dofX.segment<3>(3) = angleAxis2.axis() * angleAxis2.angle();
		Vector6d dofV = Vector6d::Zero();

		Vector6d FWithoutExtraTorque = computeSpringForce(linearStiffness, linearDamping,
									   angularStiffness, angularDamping, targetPosition, targetLinearVelocity,
									   targetRotationVector, targetAngularVelocity, dofX, dofV);

		Vector6d FWithExtraTorque = computeSpringForce(linearStiffness, linearDamping,
									angularStiffness, angularDamping, targetPosition, targetLinearVelocity,
									targetRotationVector, targetAngularVelocity, dofX, dofV, true, localAnchorPoint);

		std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
		rigidBody->setShape(m_sphere);
		SurgSim::Physics::RigidRepresentationState initialState;
		initialState.setPose(transform);
		rigidBody->setInitialState(initialState);
		SurgSim::DataStructures::Location location(localAnchorPoint);

		Vector6d F = FWithoutExtraTorque;
		rigidBody->addExternalGeneralizedForce(location, F);
		F = rigidBody->getExternalGeneralizedForce().unsafeGet();

		EXPECT_LE((F - FWithExtraTorque).cwiseAbs().maxCoeff(), 1e-7);
	}
}

TEST_F(RigidRepresentationTest, NoForceTorqueTest)
{
	// Create the rigid body
	std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");

	// Setup phase
	rigidBody->setLocalActive(true);
	rigidBody->setIsGravityEnabled(false);
	rigidBody->setDensity(m_density);
	rigidBody->setShape(m_sphere);

	// Run few time steps
	for (int timeStep = 0; timeStep < m_maxNumSimulationStepTest; timeStep++)
	{
		rigidBody->beforeUpdate(m_dt);
		rigidBody->update(m_dt);
		rigidBody->afterUpdate(m_dt);
	}

	const Vector3d    G = rigidBody->getCurrentState().getPose().translation();
	const Matrix33d&  R = rigidBody->getCurrentState().getPose().linear();
	const Quaterniond q = Quaterniond(R);
	const Vector3d    v = rigidBody->getCurrentState().getLinearVelocity();
	const Vector3d    w = rigidBody->getCurrentState().getAngularVelocity();
	ASSERT_EQ(Vector3d::Zero(), G);
	ASSERT_TRUE(q.isApprox(Quaterniond::Identity()));
	ASSERT_EQ(Vector3d::Zero(), v);
	ASSERT_EQ(Vector3d::Zero(), w);
}

TEST_F(RigidRepresentationTest, GravityTest)
{
	// Create the rigid body
	std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
	Vector3d gravity(0.0, -9.81, 0.0);

	// Setup phase
	rigidBody->setLocalActive(true);
	rigidBody->setIsGravityEnabled(true);
	rigidBody->setDensity(m_density);
	rigidBody->setShape(m_sphere);

	// Run few time steps
	for (int timeStep = 0; timeStep < m_maxNumSimulationStepTest; timeStep++)
	{
		rigidBody->beforeUpdate(m_dt);
		rigidBody->update(m_dt);
		rigidBody->afterUpdate(m_dt);

		const Vector3d    G = rigidBody->getCurrentState().getPose().translation();
		const Matrix33d&  R = rigidBody->getCurrentState().getPose().linear();
		const Quaterniond q = Quaterniond(R);
		const Vector3d    v = rigidBody->getCurrentState().getLinearVelocity();
		const Vector3d    w = rigidBody->getCurrentState().getAngularVelocity();

		const Vector3d    Gprev = rigidBody->getPreviousState().getPose().translation();
		const Vector3d    vprev = rigidBody->getPreviousState().getLinearVelocity();

		// Implicit numerical integration gives v(t+dt) = v(t) + dt.a(t+dt) = v(t) + dt.g
		//                                      p(t+dt) = p(t) + dt.v(t+dt) = p(t) + dt.v(t) + dt^2.g
		Vector3d tmpV = vprev + gravity * m_dt;
		double diffV = (v - tmpV).norm();
		Vector3d tmpG = Gprev + tmpV * m_dt;
		double diffG = (G - tmpG).norm();

		ASSERT_NEAR(0.0, diffG, epsilon);
		ASSERT_TRUE(q.isApprox(Quaterniond::Identity()));
		ASSERT_NEAR(0.0, diffV, epsilon);
		ASSERT_EQ(Vector3d::Zero(), w);
	}
}

TEST_F(RigidRepresentationTest, PreviousStateDifferentFromCurrentTest)
{
	// Create the rigid body
	std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
	Vector3d gravity(0.0, -9.81, 0.0);

	// Setup phase
	rigidBody->setLocalActive(true);
	rigidBody->setIsGravityEnabled(true);
	rigidBody->setDensity(m_density);
	rigidBody->setShape(m_sphere);

	// Run few time steps
	for (int timeStep = 0; timeStep < m_maxNumSimulationStepTest; timeStep++)
	{
		rigidBody->beforeUpdate(m_dt);
		rigidBody->update(m_dt);
		rigidBody->afterUpdate(m_dt);

		ASSERT_NE(rigidBody->getPreviousState(), rigidBody->getCurrentState());
	}
}

void disableWhenDivergeTest(std::shared_ptr<RigidRepresentation> rigidBody,
							const RigidRepresentationState& state, double dt)
{
	// Setup phase
	rigidBody->setLocalActive(true);
	rigidBody->setIsGravityEnabled(true);
	rigidBody->setInitialState(state);

	// Run 1 time step and make sure that the rigid body has been disabled
	// The rotation explode under the angular velocity too strong !
	{
		ASSERT_TRUE(rigidBody->isActive());

		rigidBody->beforeUpdate(dt);
		rigidBody->update(dt);
		rigidBody->afterUpdate(dt);

		ASSERT_FALSE(rigidBody->isActive());
	}
}

TEST_F(RigidRepresentationTest, DisableWhenDivergeTest)
{
	Quaterniond q  = Quaterniond::Identity();
	Vector3d vZero = Vector3d::Zero();
	Vector3d vMax  = Vector3d::Constant(std::numeric_limits<double>::max());

	// Test linear failure (with Signaling Nan)
	{
		// Create the rigid body
		std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
		rigidBody->setDensity(m_density);
		rigidBody->setShape(m_sphere);

		// Sets the state
		m_state.reset();
		m_state.setPose(SurgSim::Math::makeRigidTransform(q, vMax));
		m_state.setLinearVelocity(Vector3d::Constant(std::numeric_limits<double>::signaling_NaN()));

		SCOPED_TRACE("Testing linear with Signaling Nan");
		disableWhenDivergeTest(rigidBody, m_state, m_dt);
	}

	// Test linear failure (with quiet Nan)
	{
		// Create the rigid body
		std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
		rigidBody->setDensity(m_density);
		rigidBody->setShape(m_sphere);

		// Sets the state
		m_state.reset();
		m_state.setPose(makeRigidTransform(q, vMax));
		m_state.setLinearVelocity(Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));

		SCOPED_TRACE("Testing linear with Quiet Nan");
		disableWhenDivergeTest(rigidBody, m_state, m_dt);
	}

	// Test linear failure (with numerical maximum value)
	{
		// Create the rigid body
		std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
		rigidBody->setDensity(m_density);
		rigidBody->setShape(m_sphere);

		// Sets the state
		m_state.reset();
		m_state.setPose(makeRigidTransform(q, vMax));
		m_state.setLinearVelocity(Vector3d::Constant(std::numeric_limits<double>::max()));

		SCOPED_TRACE("Testing linear with double max");
		disableWhenDivergeTest(rigidBody, m_state, m_dt);
	}

	// Test angular failure (with Signaling Nan)
	{
		// Create the rigid body
		std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
		rigidBody->setDensity(m_density);
		rigidBody->setShape(m_sphere);

		// Sets the state
		m_state.reset();
		m_state.setPose(makeRigidTransform(q, vZero));
		m_state.setAngularVelocity(Vector3d::Constant(std::numeric_limits<double>::signaling_NaN()));

		SCOPED_TRACE("Testing angular with Signaling Nan");
		disableWhenDivergeTest(rigidBody, m_state, m_dt);
	}

	// Test angular failure (with quiet Nan)
	{
		// Create the rigid body
		std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
		rigidBody->setDensity(m_density);
		rigidBody->setShape(m_sphere);

		// Sets the state
		m_state.reset();
		m_state.setPose(makeRigidTransform(q, vZero));
		m_state.setAngularVelocity(Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));

		SCOPED_TRACE("Testing angular with Quiet Nan");
		disableWhenDivergeTest(rigidBody, m_state, m_dt);
	}

	// Test angular failure (with numerical maximum value)
	{
		// Create the rigid body
		std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
		rigidBody->setDensity(m_density);
		rigidBody->setShape(m_sphere);

		// Sets the state
		m_state.reset();
		m_state.setPose(makeRigidTransform(q, vZero));
		m_state.setAngularVelocity(Vector3d::Constant(std::numeric_limits<double>::max()));

		SCOPED_TRACE("Testing angular with double max");
		disableWhenDivergeTest(rigidBody, m_state, m_dt);
	}
}

TEST_F(RigidRepresentationTest, LocalizationCreation)
{
	std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
	Location loc(Vector3d(3.0, 2.0, 1.0));

	std::shared_ptr<Localization> localization = rigidBody->createLocalization(loc);
	localization->setRepresentation(rigidBody);

	Vector3d globalPos = rigidBody->getCurrentState().getPose() * loc.rigidLocalPosition.getValue();

	EXPECT_TRUE(globalPos.isApprox(localization->calculatePosition(0.0)));
	EXPECT_TRUE(globalPos.isApprox(localization->calculatePosition(1.0)));

}

TEST_F(RigidRepresentationTest, InvalidShapes)
{
	std::shared_ptr<Shape> shapeWithNoVolume = std::make_shared<PlaneShape>();
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();

	{
		std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
		rigidBody->setShape(shapeWithNoVolume);

		std::shared_ptr<Component> component = rigidBody;
		EXPECT_THROW(component->initialize(runtime), SurgSim::Framework::AssertionFailure);
	}
}

TEST_F(RigidRepresentationTest, WithMeshShape)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>("config.txt");

	std::shared_ptr<SurgSim::Math::MeshShape> shape = std::make_shared<SurgSim::Math::MeshShape>();
	shape->load("MeshShapeData/staple_collision.ply");
	EXPECT_TRUE(shape->isValid());

	std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");
	rigidBody->setShape(shape);

	EXPECT_NO_THROW(rigidBody->initialize(runtime));
	EXPECT_TRUE(shape->isValid());

	std::shared_ptr<RigidRepresentation> rigidBody2 = std::make_shared<RigidRepresentation>("Rigid2");
	rigidBody2->setShape(shape);
	EXPECT_NO_THROW(rigidBody2->initialize(runtime));
}

TEST_F(RigidRepresentationTest, CollisionRepresentationTest)
{
	auto rigidBody(std::make_shared<RigidRepresentation>("Rigid"));

	auto collision1(std::make_shared<RigidCollisionRepresentation>("collision1"));
	auto collision2(std::make_shared<RigidCollisionRepresentation>("collision2"));

	EXPECT_EQ(nullptr, rigidBody->getCollisionRepresentation());
	EXPECT_NO_THROW(rigidBody->setCollisionRepresentation(collision1));
	EXPECT_EQ(collision1, rigidBody->getCollisionRepresentation());
	EXPECT_EQ(rigidBody, collision1->getRigidRepresentation());

	// Change the collision object
	EXPECT_NO_THROW(rigidBody->setCollisionRepresentation(collision2));
	EXPECT_EQ(collision2, rigidBody->getCollisionRepresentation());
	EXPECT_EQ(rigidBody, collision2->getRigidRepresentation());
	EXPECT_EQ(nullptr, collision1->getRigidRepresentation());

	// Clear the collision representation
	EXPECT_NO_THROW(rigidBody->setCollisionRepresentation(nullptr));
	EXPECT_EQ(nullptr, rigidBody->getCollisionRepresentation());
	EXPECT_EQ(nullptr, collision1->getRigidRepresentation());
	EXPECT_EQ(nullptr, collision2->getRigidRepresentation());
}

TEST_F(RigidRepresentationTest, DensityWithSphereShapeTest)
{
	std::shared_ptr<RigidRepresentation> rigidBody = std::make_shared<RigidRepresentation>("Rigid");

	// Mass density
	rigidBody->setDensity(m_density);

	// Shape
	rigidBody->setShape(m_sphere);

	// Test inertia
	EXPECT_TRUE(rigidBody->getLocalInertia().isApprox(m_inertia));

	// Test mass
	EXPECT_EQ(m_mass, rigidBody->getMass());

	// Test mass center
	EXPECT_TRUE(rigidBody->getMassCenter().isZero());
}


TEST_F(RigidRepresentationTest, SerializationTest)
{
	{
		SCOPED_TRACE("Encode/Decode as shared_ptr<>, should be OK");
		auto rigidRepresentation = std::make_shared<RigidRepresentation>("TestRigidRepresentation");
		YAML::Node node;
		ASSERT_NO_THROW(
			node = YAML::convert<std::shared_ptr<SurgSim::Framework::Component>>::encode(rigidRepresentation));

		std::shared_ptr<RigidRepresentation> newRepresentation;
		EXPECT_NO_THROW(newRepresentation =	std::dynamic_pointer_cast<RigidRepresentation>
											(node.as<std::shared_ptr<SurgSim::Framework::Component>>()));
	}

	{
		SCOPED_TRACE("Encode a RigidRepresentation object without a shape, should throw.");
		auto rigidRepresentation = std::make_shared<RigidRepresentation>("TestRigidRepresentation");
		auto rigidCollisionRepresentation =
			std::make_shared<RigidCollisionRepresentation>("RigidCollisionRepresentation");

		rigidRepresentation->setCollisionRepresentation(rigidCollisionRepresentation);

		EXPECT_ANY_THROW(YAML::convert<SurgSim::Framework::Component>::encode(*rigidRepresentation));
	}

	{
		SCOPED_TRACE("Encode a RigidRepresentation object with valid RigidCollisionRepresentation and shape, no throw");
		auto rigidRepresentation = std::make_shared<RigidRepresentation>("TestRigidRepresentation");
		auto rigidCollisionRepresentation =
			std::make_shared<RigidCollisionRepresentation>("RigidCollisionRepresentation");

		rigidRepresentation->setCollisionRepresentation(rigidCollisionRepresentation);
		rigidRepresentation->setDensity(0.1);
		rigidRepresentation->setLinearDamping(0.2);
		rigidRepresentation->setAngularDamping(0.3);
		rigidRepresentation->setShape(m_sphere);

		YAML::Node node;
		EXPECT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*rigidRepresentation));

		std::shared_ptr<RigidRepresentation> newRepresentation;
		newRepresentation =
			std::dynamic_pointer_cast<RigidRepresentation>(node.as<std::shared_ptr<SurgSim::Framework::Component>>());
		EXPECT_NE(nullptr, newRepresentation->getCollisionRepresentation());
		EXPECT_NEAR(0.1, newRepresentation->getValue<double>("Density"), epsilon);
		EXPECT_NEAR(0.2, newRepresentation->getValue<double>("LinearDamping"), epsilon);
		EXPECT_NEAR(0.3, newRepresentation->getValue<double>("AngularDamping"), epsilon);

		// Shape is encoded/decoded as concrete object instead of reference/shared_ptr<>.
		EXPECT_NE(m_sphere, newRepresentation->getValue<std::shared_ptr<SurgSim::Math::Shape>>("Shape"));

		auto decodedShape = newRepresentation->getValue<std::shared_ptr<SurgSim::Math::Shape>>("Shape");
		EXPECT_EQ(m_sphere->getClassName(), decodedShape->getClassName());
		EXPECT_NEAR(m_sphere->getVolume(), decodedShape->getVolume(), epsilon);

		auto newCollisionRepresentation =
			std::dynamic_pointer_cast<RigidCollisionRepresentation>(newRepresentation->getCollisionRepresentation());
		EXPECT_EQ(newRepresentation, newCollisionRepresentation->getRigidRepresentation());
	}
}

}; // namespace Physics
}; // namespace SurgSim
